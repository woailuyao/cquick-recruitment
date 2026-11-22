"""单目场地自定位主程序（左右分屏）。

处理链路：
    YOLO 检测 L/T/X  →  交点几何细化（bbox 中心不等于交点）
    →  去畸变 + 地面投影（载体系米制坐标）
    →  与 config/field.yaml 匹配（枚举同类组合 + 加权 SE(2)）
    →  输出 x / y / yaw、marker_count、residual、valid、status

界面：左侧实时 RGB（足球框 + 标志框 + 距离），右侧场地俯视图（相机光心 + 朝向箭头 + 读数）。

两种标志来源：
  1) 模型模式（推荐）：--marker-model <best.pt> --marker-map X,T,L
     每帧自动检测，无需人工点选；
  2) 手点模式（无模型时的备用）：第一帧点选，之后模板匹配跟踪，按 `c` 可重新点选。

用法：
    # 实时相机 + 标志模型
    python split_view.py --source 0 \
        --marker-model .../exp2/weights/best.pt --marker-map X,T,L \
        --ball-model .../best.pt --conf 0.85

    # 视频回放（同样的处理链路）
    python split_view.py --source xxx.mp4 --marker-model ... --marker-map X,T,L

    # 发布 ROS2 话题
    python split_view.py --source 0 --marker-model ... --ros2

按键：
    q      退出
    c      重新点选标志（手点模式）
    r      旋转地图显示（仅影响绘图，不改坐标）
    1~4    切换 field.yaml → 官方坐标 的换算方式
"""
import argparse
import sys
import json
from pathlib import Path
import math
import os
import time

import cv2
import numpy as np

from geometry import GroundProjector, solve_pose_with_matching
from pick_marker import refine_intersection
from viz import MapView, flat_markers, load_camera, load_field
from imgio import imread, project_root

TYPE_KEYS = {"1": "L", "2": "T", "3": "X"}


class Clicker:
    """在第一帧上点选标志交点（复用了 pick_marker 的细化）。"""

    def __init__(self, img):
        self.img = img
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.type = "L"
        self.points = []

    def run(self):
        win = "pick markers (1=L 2=T 3=X, s=save, q=cancel)"

        def redraw():
            disp = self.img.copy()
            for p in self.points:
                cv2.circle(disp, (int(p["u"]), int(p["v"])), 7, (0, 0, 255), 2)
                cv2.putText(disp, p["type"], (int(p["u"]) + 9, int(p["v"]) - 9),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(disp, f"type={self.type}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow(win, disp)

        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                xi, yi, how = refine_intersection(self.gray, x, y)
                self.points.append({"type": self.type, "u": xi, "v": yi, "how": how})
                print(f"  + {self.type} ({xi:.2f},{yi:.2f}) [{how}]")
                redraw()

        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win, on_mouse)
        redraw()
        while True:
            k = cv2.waitKey(30) & 0xFF
            if k in (ord(c) for c in TYPE_KEYS):
                self.type = TYPE_KEYS[chr(k)]
                redraw()
            elif k == ord("u") and self.points:
                self.points.pop()
                redraw()
            elif k == ord("s"):
                cv2.destroyWindow(win)
                return self.points
            elif k == ord("q"):
                cv2.destroyWindow(win)
                return []
        cv2.destroyWindow(win)


class Tracker:
    """模板匹配 + 光流双保险跟踪标志交点。

    为什么不是纯光流：相机平移时 LK 会把点"粘"到旁边相似的草皮上，
    表现为标注点飘离白线。这里在点选时把交点周围的小图样存成模板，
    每帧在预测位置附近做归一化互相关匹配，匹配到的是**同一块图样**，
    所以点会一直压在同一个交点上。

    流程：LK 预测 -> 模板匹配校正（score 达标则采用）-> 都不行则标记跟丢。
    """

    TPL = 40            # 模板边长（像素）
    SRCH = 64           # 搜索范围（比模板多出的边长）
    SCORE = 0.40        # 模板匹配最低得分
    MAX_JUMP = 45.0     # 单帧最大位移
    LOST_LIMIT = 15

    def __init__(self, points, gray):
        self.pts = np.array([[p["u"], p["v"]] for p in points], np.float32).reshape(-1, 1, 2)
        self.types = [p["type"] for p in points]
        self.prev = gray
        self.miss = np.zeros(len(points), np.int32)
        self.usable = np.ones(len(points), bool)
        self.tpl = []
        h = self.TPL // 2
        H, W = gray.shape[:2]
        for (u, v) in self.pts.reshape(-1, 2):
            ui, vi = int(round(u)), int(round(v))
            x0, y0 = max(0, ui - h), max(0, vi - h)
            patch = gray[y0:y0 + self.TPL, x0:x0 + self.TPL]
            self.tpl.append(patch.copy() if patch.shape[:2] == (self.TPL, self.TPL) else None)

    def _lk(self, a, b, p):
        return cv2.calcOpticalFlowPyrLK(
            a, b, p, None, winSize=(31, 31), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))

    def _match(self, gray, idx, u, v):
        """在 (u,v) 附近搜索模板，返回 (x, y, score) 或 None。"""
        t = self.tpl[idx]
        if t is None:
            return None
        h = self.TPL // 2
        pad = self.SRCH // 2
        x0 = int(round(u)) - h - pad
        y0 = int(round(v)) - h - pad
        H, W = gray.shape[:2]
        x1, y1 = x0 + self.TPL + 2 * pad, y0 + self.TPL + 2 * pad
        if x0 < 0 or y0 < 0 or x1 > W or y1 > H:
            return None
        roi = gray[y0:y1, x0:x1]
        if roi.shape[0] < t.shape[0] or roi.shape[1] < t.shape[1]:
            return None
        res = cv2.matchTemplate(roi, t, cv2.TM_CCOEFF_NORMED)
        _, mx, _, mloc = cv2.minMaxLoc(res)
        return x0 + mloc[0] + h, y0 + mloc[1] + h, float(mx)

    def update(self, gray, bgr=None):
        n = len(self.pts)
        if n == 0:
            return self.pts.reshape(-1, 2), np.zeros(0, bool)

        fwd, st1, _ = self._lk(self.prev, gray, self.pts)
        pred = self.pts.reshape(-1, 2).copy()
        if fwd is not None and st1 is not None:
            st = st1.reshape(-1) == 1
            pred[st] = fwd[st].reshape(-1, 2)

        new = self.pts.copy()
        ok = np.zeros(n, bool)
        for i, (u, v) in enumerate(pred):
            m = self._match(gray, i, u, v)
            if m is not None and m[2] >= self.SCORE and math.hypot(m[0] - u, m[1] - v) < self.SRCH:
                new[i] = (m[0], m[1])
                ok[i] = True
            elif st1 is not None and st1.reshape(-1)[i] == 1:
                new[i] = (u, v)
                ok[i] = True
            # 否则保持上一帧位置并计入丢失

        jump = np.linalg.norm((new - self.pts).reshape(-1, 2), axis=1)
        ok &= jump < self.MAX_JUMP

        self.miss[~ok] += 1
        self.miss[ok] = 0
        usable = self.miss <= self.LOST_LIMIT
        self.usable = usable

        # 匹配成功时用新位置刷新模板（适应尺度/光照的缓慢变化）
        h = self.TPL // 2
        H, W = gray.shape[:2]
        for i in np.where(ok)[0]:
            u, v = new[i].ravel()
            x0, y0 = int(round(u)) - h, int(round(v)) - h
            if x0 >= 0 and y0 >= 0 and x0 + self.TPL <= W and y0 + self.TPL <= H:
                self.tpl[i] = gray[y0:y0 + self.TPL, x0:x0 + self.TPL].copy()

        self.pts = new
        self.prev = gray
        return self.pts.reshape(-1, 2), usable

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0", help="视频路径或相机编号")
    ap.add_argument("--clicks", default=None, help="点选结果 json")
    ap.add_argument("--save-clicks", default=None)
    BASE = project_root(__file__)
    ap.add_argument("--camera", default=str(BASE / "config" / "camera.yaml"))
    ap.add_argument("--field", default=str(BASE / "config" / "field.yaml"))
    ap.add_argument("--record", default=None, help="把分屏结果录成 mp4（可选）")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--ball-model", default=None, help="YOLO 权重（如 D:\\week3_4_dailu\\best.pt），给了就检测足球")
    ap.add_argument("--conf", type=float, default=0.35, help="足球置信度阈值")
    ap.add_argument("--no-pick", action="store_true", help="不点选标志（只测足球检测时用）")
    ap.add_argument("--start-frame", type=int, default=0, help="从第几帧开始")
    ap.add_argument("--map-rot", type=int, default=90, choices=[0, 90, 180, 270],
                    help="地图朝向：0=+X右 +Y上, 90=+X上 +Y左, 180=+X左 +Y下, 270=+X下 +Y右")
    ap.add_argument("--marker-model", default=None, help="L/T/X 标志检测模型（给了就不用人工点选）")
    ap.add_argument("--marker-conf", type=float, default=0.35, help="标志检测置信度阈值")
    ap.add_argument("--marker-map", default="X,T,L",
                    help="模型类别 id 0,1,2 对应的真实类别，逗号分隔，例如 X,T,L")
    ap.add_argument("--size", default="", help="采集分辨率 WxH，例如 640x480（留空用配置里的 1280x720）")
    ap.add_argument("--fps", type=float, default=0, help="相机帧率（0=不设置；WSL 转发相机建议 10~15）")
    ap.add_argument("--loop", action="store_true", help="循环播放视频（不退出，便于验证 ROS2 话题）")
    ap.add_argument("--ros2", action="store_true", help="发布 ROS2 话题 /soccer/ball 与 /localization/field_pose")
    args = ap.parse_args()

    cam = load_camera(args.camera)
    markers_map, bounds = load_field(args.field, cam.get("field_frame"))
    flat = flat_markers(markers_map)
    proj = GroundProjector(cam["K"], cam["height_m"], cam["pitch_deg"],
                           cam["roll_deg"], cam["yaw_deg"],
                           max_range_m=float(cam["proj"].get("max_range_m", 6.0)),
                           min_range_m=float(cam["proj"].get("min_range_m", 0.3)))
    max_res = float(cam["loc"].get("max_residual_m", 0.6))

    ros2_pub = None
    if getattr(args, "ros2", False):
        from ros2_pub import create_publisher_or_none
        ros2_pub = create_publisher_or_none()

    # 四种 field.yaml → 官方坐标 的换算方式，运行中用 1/2/3/4 切换
    FRAME_OPTS = ["x,y", "y,-x", "-x,-y", "-y,x"]

    # 没显式给 --marker-model / --ball-model 时，用 config/camera.yaml 里的默认路径
    _models = cam.get("models") or {}
    if not args.marker_model and _models.get("marker") and os.path.exists(str(_models["marker"])):
        args.marker_model = str(_models["marker"])
        print(f"[INFO] 使用配置里的默认标志模型: {args.marker_model}")
    if not args.ball_model and _models.get("ball") and os.path.exists(str(_models["ball"])):
        args.ball_model = str(_models["ball"])
        print(f"[INFO] 使用配置里的默认足球模型: {args.ball_model}")

    marker_model = None
    if args.marker_model:
        from ultralytics import YOLO
        marker_model = YOLO(args.marker_model)
        print(f"[INFO] 标志检测模型已加载: {args.marker_model}（无需人工点选）")

    ball_model = None
    if args.ball_model:
        try:
            from ultralytics import YOLO
            ball_model = YOLO(args.ball_model)
            print(f"[INFO] 足球模型已加载: {args.ball_model}")
        except Exception as e:
            print(f"[WARN] 足球模型加载失败: {e}")

    # 打开视频源
    if args.source.isdigit():
        # Windows 用 DirectShow；Linux/WSL 用 V4L2（CAP_DSHOW 在 Linux 上不存在）
        if sys.platform.startswith("win"):
            cap = cv2.VideoCapture(int(args.source), cv2.CAP_DSHOW)
        else:
            cap = cv2.VideoCapture(int(args.source), cv2.CAP_V4L2)
            if cap.isOpened():
                # UVC 相机默认常用 YUYV，OpenCV 解码错会整屏发绿；强制 MJPG 最稳
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                # WSL 的 USB-over-IP 带宽有限，帧率高会撕裂；这里限制帧率
                if getattr(args, "fps", 0) and args.fps > 0:
                    cap.set(cv2.CAP_PROP_FPS, float(args.fps))
                # 只用 1 帧缓冲，避免取到陈旧帧
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    else:
        cap = cv2.VideoCapture(args.source)
    if not cap.isOpened():
        print(f"[ERROR] 打不开视频源 {args.source}")
        return
    want_w, want_h = cam["width"], cam["height"]
    if getattr(args, "size", ""):
        try:
            want_w, want_h = [int(v) for v in args.size.lower().split("x")]
        except Exception:
            print(f"[WARN] --size 格式应为 640x480，收到 {args.size}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, want_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, want_h)

    if args.start_frame > 0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, args.start_frame)
    ok, frame = cap.read()
    if not ok:
        print("[ERROR] 读不到第一帧")
        return

    # 若实际采集分辨率与标定分辨率不同（例如为了降低 USB 带宽改成 640x480），
    # 按比例缩放内参：fx/fy/cx/cy 线性缩放；畸变系数工作在归一化坐标，与尺度无关
    K_use = cam["K"].copy()
    fh, fw = frame.shape[:2]
    sx, sy = fw / float(cam["width"]), fh / float(cam["height"])
    if abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3:
        K_use[0, 0] *= sx
        K_use[0, 2] *= sx
        K_use[1, 1] *= sy
        K_use[1, 2] *= sy
        proj = GroundProjector(K_use, cam["height_m"], cam["pitch_deg"],
                               cam["roll_deg"], cam["yaw_deg"],
                               max_range_m=float(cam["proj"].get("max_range_m", 6.0)),
                               min_range_m=float(cam["proj"].get("min_range_m", 0.3)))
        print(f"[INFO] 采集 {fw}x{fh} != 标定 {cam['width']}x{cam['height']}，"
              f"内参已按 ({sx:.3f}, {sy:.3f}) 缩放")

    # 点选：优先用已有 json，否则现场点
    if args.clicks and os.path.exists(args.clicks):
        with open(args.clicks, "r", encoding="utf-8") as f:
            points = json.load(f)["points"]
        for p in points:                       # 用当前帧重新细化一次
            p["u"], p["v"], p["how"] = refine_intersection(frame, p["u"], p["v"])
        print(f"[INFO] 从 {args.clicks} 读入 {len(points)} 个点")
    elif marker_model is not None:
        points = []
        print("[INFO] 模型模式：不点选标志，每帧自动检测 L/T/X")
    elif args.no_pick:
        points = []
        print("[INFO] --no-pick：不点选标志，只跑足球检测")
    else:
        points = Clicker(frame).run()
        if not points:
            print("[WARN] 没有点选任何标志")
    if args.save_clicks and points:
        os.makedirs(os.path.dirname(args.save_clicks) or ".", exist_ok=True)
        with open(args.save_clicks, "w", encoding="utf-8") as f:
            json.dump({"points": points}, f, ensure_ascii=False, indent=2)
        print(f"[SAVE] {args.save_clicks}")

    tracker = Tracker(points, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    map_view = MapView(bounds, size=(frame.shape[1], frame.shape[0]), rot=args.map_rot)
    print(f"[INFO] 已固定地图状态：field_frame={cam.get('field_frame', 'identity')}, rot={map_view.rot}")

    writer = None
    if args.record:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.record, fourcc, 20.0,
                                 (frame.shape[1] * 2, frame.shape[0]))

    smoothed = None
    prev_match = None      # 上一帧的匹配对应关系（用于连续性约束）
    stable_key, stable_cnt = None, 0   # 同一组匹配连续出现的帧数
    _win_sized = False
    n = 0
    t0 = time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            if getattr(args, "loop", False):
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            break
        n += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        obs = []
        if marker_model is not None:
            # —— 模型检测路线：每帧自动识别 L/T/X，不需要人工点选 ——
            r = marker_model.predict(frame, imgsz=640, conf=args.marker_conf, verbose=False)[0]
            CLS2TYPE = {i: ch.strip().upper()[0]
                        for i, ch in enumerate(args.marker_map.split(","))}
            for b in r.boxes:
                cid = int(b.cls[0])
                t = CLS2TYPE.get(cid)
                if t is None or t == "P":
                    continue
                x1, y1, x2, y2 = [float(x) for x in b.xyxy[0].tolist()]
                cu, cvv = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                # 检测框中心 != 几何交点，必须做几何细化
                xi, yi, how = refine_intersection(frame, cu, cvv, max_shift=30.0, iters=300)
                if how != "center":
                    xi, yi = cu, cvv
                und = cv2.undistortPoints(np.array([[[xi, yi]]], np.float32),
                                          K_use, cam["dist"], P=K_use)[0, 0]
                g = proj.ground_from_pixel(float(und[0]), float(und[1]))
                if g is not None:
                    obs.append({"type": t, "x": g[0], "y": g[1],
                                "u": float(xi), "v": float(yi),
                                "conf": float(b.conf[0])})
        else:
            uv, usable = tracker.update(gray, frame)
            for (u, v), t, ok_flag in zip(uv, tracker.types, usable):
                if not ok_flag:
                    continue
                und = cv2.undistortPoints(np.array([[[u, v]]], np.float32),
                                          K_use, cam["dist"], P=K_use)[0, 0]
                g = proj.ground_from_pixel(float(und[0]), float(und[1]))
                if g is not None:
                    obs.append({"type": t, "x": g[0], "y": g[1], "u": float(u), "v": float(v)})

        res = solve_pose_with_matching(obs, flat, max_residual_m=max_res,
                                       max_ambiguity_margin=float(cam["loc"].get("ambiguity_margin_m", 0.20)),
                                       max_ambiguity_pose=float(cam["loc"].get("ambiguity_pose_m", 0.5)),
                                       max_ambiguity_yaw=float(cam["loc"].get("ambiguity_yaw_deg", 15.0)),
                                       max_range_m=float(cam["proj"].get("max_range_m", 6.0)),
                                       include_ambiguous_candidates=True)

        # —— 匹配连续性约束（细则 2.10 允许"与上一帧结果融合"）——
        # 场地左右近似对称，两个镜像解残差接近时会来回跳。若上一帧用的那组对应关系
        # 在本帧的残差与最优解相差不超过 HYST_MARGIN，就继续用它，避免坐标跳变。
        HYST_MARGIN = 0.12
        try:
            cands_now = res.get("candidates") or []
            if prev_match is not None and cands_now:
                best_rms = cands_now[0]["residual"]
                for c in cands_now:
                    key = frozenset((round(a["map"]["x"], 2), round(a["map"]["y"], 2))
                                    for a in c["assignments"])
                    if key == prev_match and c["residual"] <= best_rms + HYST_MARGIN:
                        res["x"], res["y"] = c["x"], c["y"]
                        res["yaw_deg"] = c["yaw_deg"]
                        res["residual"] = c["residual"]
                        res["assignments"] = c["assignments"]
                        if c["residual"] <= max_res:
                            res["valid"] = True
                            res["status"] = "ok"
                        break
            # 只有"本帧有效"的匹配才记为历史，避免把错解固化
            if res.get("valid") and res.get("assignments"):
                prev_match = frozenset((round(a["map"]["x"], 2), round(a["map"]["y"], 2))
                                       for a in res["assignments"])
        except Exception:
            pass

        # —— 多帧一致性确认 ——
        # 场地对称时两个镜像解残差接近，单帧分不开；但真实位置在连续帧里是稳定的，
        # 而错解会在两解之间跳动。连续 12 帧都是同一组匹配且残差稳定 -> 判为收敛。
        try:
            keys = []
            for c in (res.get("candidates") or []):
                keys.append(frozenset((round(a["map"]["x"], 2), round(a["map"]["y"], 2))
                                      for a in c["assignments"]))
            cur_key = keys[0] if keys else None
            if cur_key is not None and cur_key == stable_key:
                stable_cnt += 1
            else:
                stable_key, stable_cnt = cur_key, 1
            if (not res.get("valid")) and res.get("status") == "ambiguous" \
                    and stable_cnt >= 12 and res.get("residual") is not None \
                    and res["residual"] <= 0.35:
                cand = (res.get("candidates") or [None])[0]
                if cand:
                    res["x"], res["y"] = cand["x"], cand["y"]
                    res["yaw_deg"] = cand["yaw_deg"]
                    res["valid"] = True
                    res["status"] = "ok"
                    res["confirmed_frames"] = stable_cnt
        except Exception:
            pass

        # 每 20 帧打印一次匹配明细：哪个观测点匹配到了地图上的哪个标志
        if n % 20 == 1 and res.get("assignments"):
            pairs = " ".join(
                f"{a['type']}({a['obs']['x']:.1f},{a['obs']['y']:.1f})->"
                f"({a['map']['x']:.1f},{a['map']['y']:.1f})"
                for a in res["assignments"])
            print(f"[MATCH] {res['status']:22s} rms={res['residual']:.3f}  {pairs}")

        if res["valid"]:
            if smoothed is None:
                smoothed = np.array([res["x"], res["y"], res["yaw_deg"]], float)
            else:  # 指数平滑，避免箭头抖动
                cur = np.array([res["x"], res["y"], res["yaw_deg"]], float)
                d = (cur[2] - smoothed[2] + 180) % 360 - 180
                cur[2] = smoothed[2] + d
                smoothed = 0.7 * smoothed + 0.3 * cur
            pose = {"x": float(smoothed[0]), "y": float(smoothed[1]),
                    "yaw_deg": float(smoothed[2] % 360.0)}
        else:
            pose = None
            smoothed = None


        # ---- 左侧画面 ----
        left = frame.copy()
        n_balls = 0
        if ball_model is not None:
            r = ball_model.predict(frame, imgsz=640, conf=args.conf, verbose=False)[0]
            for b in r.boxes:
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
                cf = float(b.conf[0])
                n_balls += 1
                cv2.rectangle(left, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(left, ((x1 + x2) // 2, (y1 + y2) // 2), 4, (0, 255, 0), -1)
                cv2.putText(left, f"ball {cf:.2f}", (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # 以 bbox 中心输出像素坐标（任务一要求的 center_u / center_v）
                cv2.putText(left, f"({(x1+x2)//2},{(y1+y2)//2})",
                            (x1, min(left.shape[0] - 6, y2 + 22)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # 先把"跟丢"的点画成灰色，让操作者知道该重新点
        try:
            for (pu, pv), ptype, okf in zip(tracker.pts.reshape(-1, 2),
                                            tracker.types, tracker.usable):
                if not okf:
                    cv2.circle(left, (int(pu), int(pv)), 9, (150, 150, 150), 2)
                    cv2.putText(left, f"{ptype} LOST", (int(pu) + 11, int(pv) - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 2)
        except Exception:
            pass
        for o in obs:
            dist = math.hypot(o["x"], o["y"])
            # 超过 2.4 m 的标志处在投影压缩区，像素误差会被放大，标成橙色提醒
            col = (0, 140, 255) if dist > 2.4 else (0, 0, 255)
            uu, vv = int(o["u"]), int(o["v"])
            box = 26                                  # 半边长度，画成方框（与参考实现风格一致）
            full = {"L": "LCross", "T": "TCross", "X": "XCross"}.get(o["type"], o["type"])
            cv2.rectangle(left, (uu - box, vv - box), (uu + box, vv + box), col, 2)
            cv2.drawMarker(left, (uu, vv), col, cv2.MARKER_CROSS, 12, 2)
            tag = f"{full} {dist:.1f}m" + ("  FAR" if dist > 2.4 else "")
            ty = vv - box - 8 if vv - box - 8 > 14 else vv + box + 18
            cv2.putText(left, tag, (uu - box, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, col, 2)
        fps = n / max(1e-6, time.time() - t0)
        cv2.putText(left, f"FPS {fps:5.1f}  {cam['width']}x{cam['height']}",
                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(left, f"balls: {n_balls}", (15, 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # ---- 右侧地图 ----
        right = map_view.render(markers_map, pose)
        # 有歧义时把候选位姿画成灰色空心点，让操作者一眼看出"可能在这几处"
        cands = res.get("candidates") or []
        for ci, cand in enumerate(cands):
            cp = map_view.to_px(cand["x"], cand["y"])
            # 第 1 个是"最优但不可信"的解，用深灰；其余用浅灰
            col = (90, 90, 90) if ci == 0 else (150, 150, 150)
            cv2.circle(right, cp, 10 if ci == 0 else 8, col, 2)
            tag = "best" if ci == 0 else f"#{ci+1}"
            cv2.putText(right, f"{tag} rms={cand['residual']:.2f}",
                        (cp[0] + 12, cp[1] + 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55 if ci == 0 else 0.5, col, 2 if ci == 0 else 1)
        info = [f"status: {res['status']}",
                f"markers: {res['marker_count']}",
                f"residual: {res['residual']:.3f} m" if res["residual"] is not None else "residual: -",
                f"valid: {res['valid']}",
                f"balls: {n_balls}"]
        if pose:
            info += [f"x = {pose['x']:.2f} m", f"y = {pose['y']:.2f} m",
                     f"yaw = {pose['yaw_deg']:.1f} deg"]
        else:
            # 解不出唯一位姿时，至少把"最优候选"的坐标显示出来（status 已标明不算有效）
            cands = res.get("candidates") or []
            if cands:
                c0 = cands[0]
                info += [f"cand x = {c0['x']:.2f} m", f"cand y = {c0['y']:.2f} m",
                         f"cand yaw = {c0['yaw_deg']:.1f} deg (not valid)"]
            elif res.get("x") is not None:
                info += [f"x = {res['x']:.2f} m", f"y = {res['y']:.2f} m"]
        # 信息面板放到右侧空白带（场地是高度受限的，右边必然留空），避免盖住左上角的标志
        box_w = 420
        box_x = right.shape[1] - box_w - 16
        box_h = 30 * len(info) + 24
        box_y = 56
        overlay = right.copy()
        cv2.rectangle(overlay, (box_x, box_y), (box_x + box_w, box_y + box_h),
                      (255, 255, 255), -1)
        cv2.addWeighted(overlay, 0.88, right, 0.12, 0, right)
        cv2.rectangle(right, (box_x, box_y), (box_x + box_w, box_y + box_h), (170, 170, 170), 1)
        for i, s in enumerate(info):
            cv2.putText(right, s, (box_x + 14, box_y + 30 + 30 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (30, 30, 30), 2)

        if ros2_pub is not None:
            ball_list = []
            if ball_model is not None:
                for b in r.boxes:
                    x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
                    ball_list.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0, float(b.conf[0])))
            try:
                ros2_pub.publish(ball_list, pose, res, fps)
            except Exception as e:
                print("[WARN] 发布失败:", e)

        combo = np.hstack([left, right])
        WIN = "monocular localization  (left: RGB | right: map)"
        # 显示用副本：缩到不超过 1600 宽。
        # 否则窗口最大化 / 系统 DPI 缩放时，OpenCV 会把 2560 宽的图放大后裁掉右侧，
        # 表现为"右边的 map 只显示一半、信息框消失"。录制的 mp4 仍用原尺寸。
        disp = combo
        if disp.shape[1] > 1600:
            kk = 1600.0 / float(disp.shape[1])
            disp = cv2.resize(disp, (1600, int(round(disp.shape[0] * kk))),
                              interpolation=cv2.INTER_AREA)
        cv2.imshow(WIN, disp)
        # 注意：必须先 imshow，窗口才存在，否则 resizeWindow 会报 NULL window
        if not _win_sized:
            try:
                cv2.resizeWindow(WIN, disp.shape[1], disp.shape[0])
            except cv2.error:
                pass
            _win_sized = True
        if writer is not None:
            writer.write(combo)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k in (ord("1"), ord("2"), ord("3"), ord("4")):
            fi = k - ord("1")
            fr = FRAME_OPTS[fi]
            try:
                markers_map, bounds = load_field(args.field, fr)
                flat = flat_markers(markers_map)
                fh2, fw2 = frame.shape[:2]
                map_view = MapView(bounds, size=(fw2, fh2), rot=map_view.rot)
                prev_match = None
                smoothed = None
                print(f"[INFO] field_frame={fr}（第 {fi+1} 种）")
            except Exception as e:
                print("[WARN] 切换坐标定义失败:", e)
        if k == ord("r"):
            map_view.rot = (map_view.rot + 90) % 360
            map_view._recompute()
            print(f"[INFO] 地图朝向 rot={map_view.rot}")
        if k == ord("c"):
            # 现场随时重新点选：移动后跟踪点跑偏时按 c 重点，不用退出重开
            print("[INFO] 重新点选标志（c）…")
            try:
                cv2.destroyWindow(WIN)
            except cv2.error:
                pass
            newp = Clicker(frame).run()
            if newp:
                points = newp
                tracker = Tracker(points, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                smoothed = None
                print(f"[INFO] 已重新载入 {len(points)} 个标志点")
        if args.max_frames and n >= args.max_frames:
            break

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()