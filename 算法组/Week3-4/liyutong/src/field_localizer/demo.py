# -*- coding: utf-8 -*-
"""
验收演示：左 RGB / 右地图 分屏

    左半  连续实时相机画面
          · 足球检测框 + 置信度 + 像素中心（任务 1）
          · L/T/X 标志检测框（任务 2 的几何特征）
          · FPS
    右半  field.yaml 地图俯视图
          · 场地线、中圈、罚球区、球门（按地图坐标绘制）
          · 全部 16 个标志点的地图位置（暗色）
          · 本帧匹配上的标志点（亮色 + id）——这就是"地图匹配"的直观证据
          · 相机光心位置（黄点）+ 朝向箭头 + 视场扇形
    底部  x_camera_map / y_camera_map / yaw_camera_map / marker_count / residual / valid

用法
    python demo.py                                  # 摄像头 0
    python demo.py --cam 1
    python demo.py --source videos/X_2.mp4          # 用视频文件调试
    python demo.py --marker-model none              # 只跑任务 1
    python demo.py --record demo.mp4                # 同时录一份
    python demo.py --headless --frames 5            # 不开窗口，验证管线
    python demo.py --cam 0 --whoami liyutong        # 合规录屏用：终端打身份横幅 + 画面打水印

按键
    q / Esc 退出      空格 暂停      s 截图
"""
import argparse
import math
import os
import time

import cv2
import numpy as np

import localize as L

COL_BOX = (0, 255, 0)
COL_CENTER = (0, 0, 255)
COL_TEXT = (0, 255, 255)
COL_OK = (0, 220, 0)
COL_BAD = (0, 0, 255)
COL_CAM = (0, 255, 255)
TYPE_COLOR = {"L": (0, 165, 255), "T": (0, 200, 0), "X": (255, 0, 255)}
TYPE_DIM = {"L": (40, 90, 130), "T": (40, 110, 40), "X": (130, 40, 130)}


# ============================================================
# 版面：场地地图
# ============================================================
class MapView:
    """把 field.yaml 画成俯视图；地图 +X 朝上（球门在顶端），+Y 朝左"""

    def __init__(self, fld, goal, size=(480, 360), pad=34):
        self.W, self.H = size
        self.fld = fld
        self.goal = goal or {}
        self.HX = float(fld["length_m"]) / 2
        self.HY = float(fld["width_m"]) / 2
        self.pad = pad
        # X（8 m）映射到竖直边，Y（6 m）映射到水平边
        self.sc = min((self.W - 2 * pad) / (2 * self.HY),
                      (self.H - 2 * pad) / (2 * self.HX))

    def px(self, x, y):
        return (int(round(self.W / 2 - y * self.sc)),
                int(round(self.H / 2 - x * self.sc)))

    def rect(self, img, x0, y0, x1, y1, col, th=1):
        cv2.rectangle(img, self.px(x0, y0), self.px(x1, y1), col, th, cv2.LINE_AA)

    def draw(self, markers, res=None, hfov_deg=65.0):
        W, H, sc = self.W, self.H, self.sc
        img = np.full((H, W, 3), 22, np.uint8)
        HX, HY = self.HX, self.HY
        fld = self.fld

        # 草地底
        cv2.rectangle(img, self.px(HX, HY), self.px(-HX, -HY), (34, 58, 34), -1)
        line = (215, 215, 215)
        # 边界 + 中线
        self.rect(img, HX, HY, -HX, -HY, line, 2)
        cv2.line(img, self.px(0, HY), self.px(0, -HY), line, 1, cv2.LINE_AA)
        # 中圈
        cv2.circle(img, self.px(0, 0), max(2, int(float(fld["center_circle_r_m"]) * sc)),
                   line, 1, cv2.LINE_AA)
        # 罚球区（两端）
        pd_ = float(fld["penalty_depth_m"])
        pw = float(fld["penalty_width_m"]) / 2
        for sgn in (1, -1):
            self.rect(img, sgn * HX, pw, sgn * (HX - pd_), -pw, line, 1)
        # 球门（在 +X 端）
        gx = float(self.goal.get("x") or HX)
        gw = float(self.goal.get("width_m") or 2.8) / 2
        cv2.line(img, self.px(gx, gw), self.px(gx, -gw), (90, 220, 255), 3, cv2.LINE_AA)

        # 地图上的标志点：先画全部（暗）
        for m in markers:
            p = self.px(m["x"], m["y"])
            cv2.drawMarker(img, p, TYPE_DIM.get(m["type"], (70, 70, 70)),
                           cv2.MARKER_SQUARE, 7, 1)

        # 相机位姿
        pose = None
        inl = []
        if res and res.get("valid"):
            pose = (res["x_camera_map"], res["y_camera_map"], res["yaw_camera_map"])
            inl = res.get("inlier_map_ids") or []

        # 匹配上的标志点：亮色 + id
        for m in markers:
            if int(m["id"]) in inl:
                p = self.px(m["x"], m["y"])
                col = TYPE_COLOR.get(m["type"], (255, 255, 255))
                cv2.circle(img, p, 5, col, -1, cv2.LINE_AA)
                cv2.circle(img, p, 7, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(img, f'{m["id"]}{m["type"]}', (p[0] + 9, p[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)

        if pose is not None:
            cx, cy, yaw = pose
            p = self.px(cx, cy)
            yr = math.radians(yaw)
            # 屏幕上 +X 向上、+Y 向左；方位角 θ 从 +X 转向 +Y
            fwd = np.array([-math.sin(yr), -math.cos(yr)])
            # 视场扇形（贴地大概是梯形，这里用两条边界线示意）
            half = math.radians(hfov_deg / 2)
            for s in (+1, -1):
                a = yr + s * half
                d = np.array([-math.sin(a), -math.cos(a)])
                q = (int(p[0] + d[0] * 2.6 * sc), int(p[1] + d[1] * 2.6 * sc))
                cv2.line(img, p, q, (70, 130, 130), 1, cv2.LINE_AA)
            tip = (int(p[0] + fwd[0] * 1.1 * sc), int(p[1] + fwd[1] * 1.1 * sc))
            cv2.arrowedLine(img, p, tip, COL_CAM, 2, cv2.LINE_AA, tipLength=0.32)
            cv2.circle(img, p, 4, COL_CAM, -1, cv2.LINE_AA)

        # 坐标与比例标注（cv2 不认中文，全用 ASCII）
        cv2.putText(img, "goal (+X)", (8, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (90, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(img, "+Y", (8, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (150, 150, 150), 1, cv2.LINE_AA)
        cv2.putText(img, "-Y", (W - 28, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (150, 150, 150), 1, cv2.LINE_AA)
        # 1 m 比例尺
        bx, by = W - 20 - int(sc), H - 26
        cv2.line(img, (W - 20 - int(sc), by), (W - 20, by), (200, 200, 200), 2)
        cv2.putText(img, "1m", (W - 20 - int(sc) + 2, by - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, "field map (field.yaml)", (8, H - 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1, cv2.LINE_AA)
        return img


# ============================================================
# 版面：左半实时画面
# ============================================================
def put_text(img, text, org, scale=0.5, color=(255, 255, 255), thick=1):
    """cv2 没有描边参数，用「先画粗黑、再画彩色」模拟。
    草地、白线、天空交替出现，纯色字总有一段看不清，加描边才稳。"""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thick, cv2.LINE_AA)


def draw_balls(img, boxes):
    for (x1, y1, x2, y2, cf) in boxes:
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        cv2.rectangle(img, (x1, y1), (x2, y2), COL_BOX, 2)
        cv2.circle(img, (cx, cy), 4, COL_CENTER, -1)
        put_text(img, f"ball {cf:.2f} ({cx},{cy})", (x1, max(14, y1 - 7)),
                 0.52, COL_BOX, 2)


def draw_markers(img, dets):
    for d in dets:
        t = d[0]
        col = TYPE_COLOR.get(t, (200, 200, 200))
        x1, y1, x2, y2 = int(d[3]), int(d[4]), int(d[5]), int(d[6])
        cf = float(d[7])
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 1)
        put_text(img, f"{t} {cf:.2f}", (x1 + 1, max(10, y1 - 2)), 0.4, col, 1)
        cv2.circle(img, (int(d[1]), int(d[2])), 2, col, -1)


# ============================================================
# 版面：底部字段条
# ============================================================
def mod_version(mod):
    try:
        return getattr(__import__(mod), "__version__", "n/a")
    except Exception:
        return "n/a"


def sys_version():
    import sys
    return sys.version.split()[0]


def self_proof_banner(name, cam_desc, ball_model, marker_model, src=None):
    """录屏自证横幅：把细则「六、防抄袭」要求的姓名 + 实时时间打到终端，
    再补上环境版本、模型路径、数据来源，让人没法把这段录屏说成别人的。"""
    import platform
    import socket
    import sys

    def ver(mod, attr="__version__"):
        try:
            return getattr(__import__(mod), attr)
        except Exception:
            return "n/a"

    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    bar = "=" * 74
    print(bar)
    print("  CQuick 视觉组 · 第 3–4 周招新考核 · 自我证明")
    # 这两行是细则第六节要求出现在录屏画面里的原始信息
    print(f"  姓名: {name}")
    print(f"  时间: {ts}")
    print("-" * 74)
    print(f"  主机: {socket.gethostname()}  {platform.system()} {platform.release()}")
    print(f"  Python: {sys.version.split()[0]}   torch: {ver('torch')}"
          f"   ultralytics: {ver('ultralytics')}")
    print(f"  输入源: {src if src else cam_desc}")
    print(f"  足球模型: {ball_model}")
    print(f"  标志模型: {marker_model}")
    print(bar)
    print("  本段录屏一镜到底，不剪切、不拼接、不用预录视频。", flush=True)
    return ts


def make_splash(lines, size=(964, 552)):
    """启动证明页：把所有自证信息画成一张图，在程序窗口里停若干秒。
    这样即使录屏软件只能录「单个窗口」（Win+G 就是这样），
    姓名 / 实时时间 / 环境 / 模型路径 也全都出现在录屏画面里。
    注意 cv2 不能画中文，这里一律 ASCII，字段名用英文。"""
    W, H = size
    img = np.full((H, W, 3), 18, np.uint8)
    cv2.rectangle(img, (0, 0), (W, 6), (0, 200, 255), -1)
    y = 62
    for i, (txt, scale, col) in enumerate(lines):
        thick = 2 if i == 0 else 1
        put_text(img, txt, (36, y), scale, col, thick)
        y += 30 if i else 46
    cv2.putText(img, "This window is being recorded live. No cuts, no playback.",
                (36, H - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1,
                cv2.LINE_AA)
    return img


def show_splash(win, img, seconds):
    """把证明页显示 seconds 秒；按任意键可提前结束。"""
    cv2.imshow(win, img)
    t_end = time.time() + seconds
    while time.time() < t_end:
        if cv2.waitKey(50) != -1:
            break
        cv2.imshow(win, img)


def stamp_view(img, name, ts):
    """在合成图右上角持续打印「姓名 + 当前时钟」，每秒刷新。
    这样即使录制者把终端窗口最小化，画面本身仍能自证身份与实时性。"""
    txt = f"{name}  {time.strftime('%Y-%m-%d %H:%M:%S')}"
    txt2 = f"start {ts}"
    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    (tw2, _), _ = cv2.getTextSize(txt2, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    x0, y0 = img.shape[1] - tw - 18, 8
    cv2.rectangle(img, (x0 - 8, y0 - 4),
                  (img.shape[1] - 6, y0 + th + th + 16), (0, 0, 0), -1)
    cv2.putText(img, txt, (x0, y0 + th), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 255), 1, cv2.LINE_AA)
    # 启动时刻固定不动，用来对照右上角时钟确实在走
    cv2.putText(img, txt2, (img.shape[1] - tw2 - 18, y0 + th + th + 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 190), 1, cv2.LINE_AA)


def panel(width, res):
    h = 96
    img = np.full((h, width, 3), 20, np.uint8)

    def fmt(v, unit="", nd=3):
        return "n/a" if v is None else f"{v:.{nd}f}{unit}"

    ok = bool(res.get("valid"))
    rows = [
        ("x_camera_map", fmt(res.get("x_camera_map"), " m")),
        ("y_camera_map", fmt(res.get("y_camera_map"), " m")),
        ("yaw_camera_map", fmt(res.get("yaw_camera_map"), " deg", 2)),   # cv2 只认 ASCII
        ("marker_count", "n/a" if res.get("marker_count") is None else str(res.get("marker_count"))),
        ("residual", fmt(res.get("residual"), " px", 2)),
        ("valid", "TRUE" if ok else "FALSE"),
    ]
    for i, (k, v) in enumerate(rows):
        cx = 12 + (i % 3) * (width // 3)
        cy = 30 + (i // 3) * 32
        cv2.putText(img, k, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (135, 135, 135),
                    1, cv2.LINE_AA)
        col = COL_OK if (k == "valid" and ok) else (COL_BAD if k == "valid" else COL_TEXT)
        cv2.putText(img, v, (cx + 126, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)

    st = res.get("status") or ""
    extra = f"status={st}"
    if ok:
        extra += (f"   resolved_by={res.get('resolved_by')}"
                  f"   inliers={res.get('inlier_map_ids')}"
                  f"   residual_ground={res.get('residual_ground_m')} m")
    else:
        extra += f"   reason={res.get('reason') or res.get('resolved_by') or '-'}"
    cv2.putText(img, extra, (12, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 165, 120),
                1, cv2.LINE_AA)
    return img


# ============================================================
# 主循环
# ============================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--source", default=None, help="视频文件路径；不给就用摄像头")
    ap.add_argument("--ball-model", default="runs_ball/baseline/weights/best.pt")
    ap.add_argument("--marker-model", default="runs_marker/baseline/weights/best.pt")
    ap.add_argument("--conf-ball", type=float, default=0.40)
    ap.add_argument("--conf-mark", type=float, default=0.35)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--height", type=float, default=None, help="覆盖相机高度（米）")
    ap.add_argument("--solve-h", action="store_true",
                    help="开机用前若干帧反解高度（需有一帧看到 ≥4 个标志点）")
    ap.add_argument("--tol", type=float, default=0.45, help="地图匹配容差（米）")
    ap.add_argument("--scale", type=float, default=0.75, help="左半幅显示缩放")
    ap.add_argument("--map-size", default="480x360", help="右侧地图尺寸")
    ap.add_argument("--record", default=None, help="录成 mp4（提交素材用）")
    ap.add_argument("--raw-rec", default=None,
                    help="录**原始相机帧** 640x480（给 eval_c0 用）。--record 录的是合成图，"
                         "不能拿去算误差")
    ap.add_argument("--record-fps", type=float, default=15.0)
    ap.add_argument("--headless", action="store_true", help="不开窗口（验证管线）")
    ap.add_argument("--frames", type=int, default=0, help="headless 处理多少帧")
    ap.add_argument("--whoami", default="",
                    help="录屏自证：填姓名拼音，终端打身份横幅、画面右上角打姓名+时钟")
    ap.add_argument("--splash", type=float, default=0.0,
                    help="配合 --whoami：在程序窗口里先显示 N 秒启动证明页（推荐 6）")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="录满 N 秒自动停止（0=一直跑到按 q），现场录素材用")
    args = ap.parse_args()

    cam_desc = (f"摄像头 index {args.cam} / 640x480 (CAP_DSHOW)")
    who = args.whoami.strip()
    start_ts = ""
    if who:
        start_ts = self_proof_banner(who, cam_desc, args.ball_model,
                                     args.marker_model, src=args.source)
    win = f"{who} | CQuick task1(RGB) | task2(map)" if who else "CQuick task1(RGB) | task2(map)"

    from ultralytics import YOLO

    K, dist = L.load_intrinsics()
    R_gc, height_m = L.load_extrinsics()
    markers, goal, fld = L.load_field()
    field_dims = (float(fld["length_m"]), float(fld["width_m"]))
    if args.height is not None:
        height_m = args.height
        print(f"高度用命令行覆盖值 {height_m:.3f} m")

    mw, mh = (int(v) for v in args.map_size.lower().split("x"))
    mv = MapView(fld, goal, (mw, mh))

    ball = None
    if os.path.exists(args.ball_model):
        ball = YOLO(args.ball_model)
        print(f"足球模型：{args.ball_model}")
    else:
        print(f"⚠️ 缺足球模型 {args.ball_model}，左半幅不画足球框")

    mk = None
    if str(args.marker_model).lower() not in ("none", "off", ""):
        if os.path.exists(args.marker_model):
            mk = YOLO(args.marker_model)
            print(f"标志模型：{args.marker_model}  类别 {mk.names}")
        else:
            print(f"⚠️ 缺标志模型 {args.marker_model}，任务 2 无法出解")

    cap = (cv2.VideoCapture(args.source) if args.source
           else cv2.VideoCapture(args.cam, cv2.CAP_DSHOW))
    if not cap.isOpened():
        print("视频源打不开：摄像头编号用 camera_check.py 确认；文件路径检查一下")
        return
    if not args.source:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    writer = None
    raw_writer = None
    prev = None
    paused = False
    view = None
    t0, nfr, fps, n_done = time.time(), 0, 0.0, 0
    h_samples = []
    t_rec = None

    print("按键：q 退出 / 空格 暂停 / s 截图")

    # ---- 启动证明页：只录这个窗口也能自证 ----
    if who and args.splash > 0 and not args.headless:
        import platform
        import socket
        src_txt = ("live camera %d / 640x480 (CAP_DSHOW)" % args.cam
                   if not args.source else args.source)
        show_splash(win, make_splash([
            ("CQuick RECRUITMENT EXAM  -  WEEK 3-4", 0.62, (0, 200, 255)),
            ("NAME: " + who, 0.80, (0, 255, 255)),
            ("TIME: " + start_ts, 0.62, (255, 255, 255)),
            ("HOST: %s   %s %s" % (socket.gethostname(), platform.system(),
                                   platform.release()), 0.50, (200, 200, 200)),
            ("PYTHON %s   TORCH %s" % (sys_version(), mod_version("torch")), 0.50,
             (200, 200, 200)),
            ("ULTRALYTICS %s" % mod_version("ultralytics"), 0.50, (200, 200, 200)),
            ("INPUT: " + src_txt, 0.50, (180, 255, 180)),
            ("BALL MODEL: " + args.ball_model, 0.46, (180, 255, 180)),
            ("MARKER MODEL: " + str(args.marker_model), 0.46, (180, 255, 180)),
            ("TASK 1 ball detect   |   TASK 2 marker map localize", 0.46,
             (150, 200, 255)),
        ]), args.splash)
    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                print("读帧结束")
                break
            if frame.shape[1] != 640 or frame.shape[0] != 480:
                frame = cv2.resize(frame, (640, 480))
            H, W = frame.shape[:2]
            nfr += 1
            if time.time() - t0 > 0.5:
                fps = nfr / (time.time() - t0)
                t0, nfr = time.time(), 0

            left = frame.copy()

            if args.raw_rec:
                if raw_writer is None:
                    raw_writer = cv2.VideoWriter(
                        args.raw_rec, cv2.VideoWriter_fourcc(*"mp4v"),
                        args.record_fps, (W, H))
                raw_writer.write(frame)

            # ---- 任务 1：足球 ----
            n_ball = 0
            if ball is not None:
                r = ball.predict(left, conf=args.conf_ball, imgsz=args.imgsz, verbose=False)[0]
                bs = [(int(b.xyxy[0][0]), int(b.xyxy[0][1]),
                       int(b.xyxy[0][2]), int(b.xyxy[0][3]), float(b.conf[0]))
                      for b in r.boxes]
                n_ball = len(bs)
                draw_balls(left, bs)

            # ---- 任务 2：标志点 ----
            dets = []
            if mk is not None:
                r2 = mk.predict(left, conf=args.conf_mark, imgsz=args.imgsz, verbose=False)[0]
                for b in r2.boxes:
                    x1, y1, x2, y2 = (int(b.xyxy[0][0]), int(b.xyxy[0][1]),
                                      int(b.xyxy[0][2]), int(b.xyxy[0][3]))
                    dets.append((mk.names[int(b.cls[0])], (x1 + x2) / 2.0, (y1 + y2) / 2.0,
                                 x1, y1, x2, y2, float(b.conf[0])))
                draw_markers(left, dets)

            det_pts = [(d[0], d[1], d[2]) for d in dets]
            res = L.localize(det_pts, K, dist, R_gc, height_m, markers, goal=goal,
                             tol_m=args.tol, require_margin=(H, W),
                             field_dims=field_dims, prev=prev)
            if res["valid"]:
                prev = dict(x=res["x_camera_map"], y=res["y_camera_map"],
                            yaw=res["yaw_camera_map"])
            elif res.get("marker_count", 0) < 2:
                prev = None      # 断了就清，避免拿旧位姿去消歧

            if args.solve_h and mk is not None:
                if len(det_pts) >= 4:
                    h_samples.append(det_pts)
                if len(h_samples) >= 12:
                    out = L.solve_height(h_samples, K, dist, R_gc, markers,
                                         goal=goal, field_dims=field_dims)
                    if out["best_h"]:
                        height_m = out["best_h"]
                        print(f"[solve-h] 反解高度 {height_m:.3f} m  {out['hint']}")
                    args.solve_h = False
                    h_samples = []

            hfov = math.degrees(2 * math.atan(W / (2 * float(K[0, 0]))))
            right = mv.draw(markers, res, hfov_deg=hfov)

            # ---- 版面信息 ----
            put_text(left, f"FPS {fps:.1f}", (10, 26), 0.75, (255, 255, 0), 2)
            put_text(left, f"TASK1 ball={n_ball}   TASK2 markers={len(dets)}",
                     (10, 52), 0.5, (235, 255, 235), 1)
            if res["valid"]:
                put_text(left, f"({res['x_camera_map']:+.2f}, {res['y_camera_map']:+.2f})"
                         f" {res['yaw_camera_map']:+.1f}deg", (10, 480 - 12),
                         0.55, COL_CAM, 2)

            sc = args.scale
            lw = cv2.resize(left, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
            sep = np.full((max(lw.shape[0], mh), 4, 3), 70, np.uint8)
            top = np.hstack([lw, sep, right])
            view = np.vstack([top, panel(top.shape[1], res)])
        if t_rec is None:
            t_rec = time.time()
        if who:
            stamp_view(view, who, start_ts)

        if view is None:
            continue

        # writer 要在 headless 分支之前建，否则 --headless --record 什么都不录
        if args.record and writer is None:
            writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"),
                                     args.record_fps, (view.shape[1], view.shape[0]))
        if writer is not None:
            writer.write(view)

        if args.seconds and t_rec is not None and time.time() - t_rec >= args.seconds:
            print(f"已录满 {args.seconds:.0f} 秒，自动停止")
            break

        if args.headless:
            n_done += 1
            if args.frames and n_done >= args.frames:
                print(f"headless 处理了 {n_done} 帧，最后一帧 {view.shape}")
                break
            continue

        cv2.imshow(win, view)
        k = cv2.waitKeyEx(1)
        if k == -1:
            continue
        kk = k & 0xFF
        if kk in (ord('q'), 27):
            break
        elif kk == 32:
            paused = not paused
            print("暂停" if paused else "继续")
        elif kk == ord('s'):
            fn = time.strftime("shot_%H%M%S.jpg")
            cv2.imwrite(fn, view)
            print("截图", fn)

    cap.release()
    if writer is not None:
        writer.release()
        print("合成画面已保存", args.record)
    if raw_writer is not None:
        raw_writer.release()
        print("原始相机帧已保存", args.raw_rec, "（这个是给 eval_c0 用的）")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
