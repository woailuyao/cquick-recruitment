# -*- coding: utf-8 -*-
"""realtime_locate.py —— 任务二实时左右分屏定位演示

左屏：相机实时 RGB 画面 + 标志检测框（L/T 类别 + 置信度）
右屏：field.yaml 俯视地图 + 相机光心图标 + 朝向箭头 + 实时状态
      x / y / yaw / z(离地高度) / pitch / marker_count / residual /
      valid / status，相机移动时地图实时更新

数据流：Windows win_cam_bridge(MJPEG HTTP) → NetCamera → YOLO 标志检测 →
        固定外参定位(locator_fixed：像素射线→地面(Z=0)→2D Kabsch，
        z/pitch/roll 全程固定) → 分屏渲染

用法（WSL 内，先在 Windows 侧启动 win_cam_bridge）：
  实时相机：
    ~/vision_ws/k1_soccer_vision/.venv/bin/python realtime_locate.py \\
        --source net://127.0.0.1:8090 \\
        --weights /mnt/d/captures/landmarks/weights/best_manual_150.pt \\
        --intr /mnt/d/captures/calib/intrinsics.yaml \\
        --extr templates/camera_extrinsics.yaml
  视频回放（无相机时验证链路）：
    ... realtime_locate.py --source /mnt/d/captures/M1.mp4 --no-show \\
        --max-frames 40 --save smoke.png
按键：q / ESC 退出；s 保存分屏截图到 --shot-dir
"""
import argparse
import math
import os
import sys
import threading
import time
from datetime import datetime
from collections import deque

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from locator import load_field, load_intrinsics, field_type  # noqa: E402
from locator_fixed import (load_extrinsics, camera_to_base,  # noqa: E402
                           pixels_to_ground, solve_pose_fixed)
try:
    from net_camera import NetCamera
except ImportError:
    NetCamera = None  # 仅 --source net:// 时需要

# ---------- 检测 ----------
_YOLO = {}


def get_model(weights):
    if weights not in _YOLO:
        from ultralytics import YOLO
        _YOLO[weights] = YOLO(weights)
    return _YOLO[weights]


def detect_boxes(model, bgr, conf, imgsz, enhance):
    """YOLO 检测，返回 [(conf, x, y, type, bgr, x1, y1, x2, y2), ...]，
    同类 35px 内去重（与 locator.detect_yolo 同规则）。"""
    if enhance == "clahe":
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = cv2.createCLAHE(2.0, (8, 8)).apply(l)
        bgr = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    res = model.predict(bgr, imgsz=imgsz, conf=conf, verbose=False)[0]
    names = {0: "L", 1: "T"}
    COL = {"L": (230, 120, 30), "T": (30, 120, 230)}   # BGR：L 蓝 / T 橙
    cand = []
    for bx in res.boxes:
        cls = int(bx.cls[0])
        c = float(bx.conf[0])
        x1, y1, x2, y2 = [float(v) for v in bx.xyxy[0]]
        cand.append((c, (x1 + x2) / 2, (y1 + y2) / 2, names.get(cls, "L"),
                     x1, y1, x2, y2))
    cand.sort(reverse=True)
    out = []
    for c, x, y, t, x1, y1, x2, y2 in cand:
        if not any(tt == t and (x - ox) ** 2 + (y - oy) ** 2 < 35 ** 2
                   for _, ox, oy, tt, *_ in out):
            out.append((c, x, y, t, COL[t], x1, y1, x2, y2))
    return out


# ---------- 地图 ----------
class TopView:
    """field.yaml 俯视地图面板（右屏）。坐标系：+X 朝球门、+Y 向左。"""

    def __init__(self, field, w=640, h=720,
                 xmin=-4.2, xmax=4.9, ymin=-3.1, ymax=3.1):
        self.field = field
        self.w, self.h = w, h
        self.x0, self.y1 = xmin, ymax
        m = 42                                    # 面板四周留白
        self.mx, self.my = m, m + 26              # 地图原点像素（面板内）
        self.s = min((w - 2 * m) / (xmax - xmin),
                     (h - 2 * m - 26) / (ymax - ymin))
        self.trail = deque(maxlen=300)

    def to_px(self, x, y):
        return (int(self.mx + (x - self.x0) * self.s),
                int(self.my + (self.y1 - y) * self.s))

    def draw(self, canvas, ox, oy, cam=None, yaw=None, valid=False):
        """把地图画到 canvas 的 (ox, oy) 偏移处。cam=(x, y) 场地坐标。"""
        img = np.full((self.h, self.w, 3), 245, np.uint8)
        cv2.putText(img, "field.yaml v2 top view   (+X -> goal)",
                    (14, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40),
                    1, cv2.LINE_AA)

        def P(x, y):
            # 注意：图形画在面板图 img 上（宽 self.w），只能用面板内坐标；
            # 整体拼到 canvas 的 (ox, oy) 由最后的切片赋值完成
            return self.to_px(x, y)

        # 1m 网格 + 刻度
        GRID, TICK = (222, 222, 222), (130, 130, 130)
        gx = math.ceil(self.x0)
        while gx <= 4.9:
            cv2.line(img, P(gx, 3.1), P(gx, -3.1), GRID, 1)
            cv2.putText(img, str(gx), P(gx - 0.08, -3.25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, TICK, 1, cv2.LINE_AA)
            gx += 1
        gy = -3
        while gy <= 3:
            cv2.line(img, P(self.x0, gy), P(4.9, gy), GRID, 1)
            cv2.putText(img, f"{gy:+d}", P(-4.15, gy - 0.06),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, TICK, 1, cv2.LINE_AA)
            gy += 1

        # 球门端线 x=3.977（粗灰线）
        cv2.line(img, P(3.977, 3.05), P(3.977, -3.05), (120, 120, 120), 5)
        # 点球点
        cv2.circle(img, P(0, 0), 4, (40, 40, 40), -1)
        cv2.putText(img, "PenaltyPoint", P(-0.30, 0.20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1,
                    cv2.LINE_AA)

        # 标志：L 蓝 / T 橙（与左屏检测框同色）；PenalityPoint 已单独画过
        for name, (x, y) in self.field.items():
            if name == "PenalityPoint":
                continue
            t = name[0] if name.startswith(("L", "T")) else "?"
            color = (230, 120, 30) if t == "L" else (30, 120, 230)
            cv2.circle(img, P(x, y), 6, color, -1)
            cv2.circle(img, P(x, y), 9, color, 1)
            cv2.putText(img, name, P(x + 0.12, y + 0.08),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1,
                        cv2.LINE_AA)

        # 轨迹 + 相机光心 + 朝向箭头
        for tx, ty in list(self.trail)[:-1]:
            cv2.circle(img, P(tx, ty), 2, (150, 150, 150), -1)
        if cam is not None:
            cv2.circle(img, P(*cam), 8, (0, 0, 220), -1)
            cv2.circle(img, P(*cam), 12, (0, 0, 220), 2)
            if yaw is not None:
                r = math.radians(yaw)
                tip = (cam[0] + 1.3 * math.cos(r), cam[1] + 1.3 * math.sin(r))
                cv2.arrowedLine(img, P(*cam), P(*tip), (0, 0, 220), 3,
                                cv2.LINE_AA, tipLength=0.25)
                # 文字放在箭头尖端外侧，避开相机图标与 LOCKED
                lx = cam[0] + 1.45 * math.cos(r) - 0.25
                ly = cam[1] + 1.45 * math.sin(r) + 0.08
                cv2.putText(img, f"yaw {yaw:+.1f} deg", P(lx, ly),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 220), 1,
                            cv2.LINE_AA)
            if valid:
                cv2.putText(img, "LOCKED", P(cam[0] - 0.35, cam[1] - 0.34),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 0), 2,
                            cv2.LINE_AA)
        canvas[oy:oy + self.h, ox:ox + self.w] = img


# ---------- 主循环 ----------
def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", default="net://127.0.0.1:8090",
                    help="net://host:port 网络相机 / 视频文件路径 / 相机序号")
    ap.add_argument("--weights", default="/mnt/d/captures/landmarks/"
                                        "weights/best_manual_150.pt")
    ap.add_argument("--field", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "templates/field.yaml"))
    ap.add_argument("--intr", default="/mnt/d/captures/calib/"
                                      "intrinsics.yaml")
    ap.add_argument("--extr", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "templates/camera_extrinsics.yaml"),
        help="固定外参（h/pitch/roll），整段不变")
    ap.add_argument("--conf", type=float, default=0.05)
    ap.add_argument("--imgsz", type=int, default=640,
                    help="与训练尺寸一致；切勿用 960/1280（白线角点误检）")
    ap.add_argument("--enhance", choices=["none", "clahe"], default="none",
                    help="默认关闭 CLAHE——它会把白线角点放建成假 L/T 标志")
    ap.add_argument("--stride", type=int, default=2,
                    help="每 N 帧解算一次位姿（检测每帧都做）")
    ap.add_argument("--smooth", type=int, default=5,
                    help="位姿平滑窗口（帧解中位数）")
    ap.add_argument("--max-frames", type=int, default=0,
                    help=">0 时只处理前 N 帧（冒烟测试）")
    ap.add_argument("--no-show", action="store_true",
                    help="不弹窗口（无显示环境冒烟测试）")
    ap.add_argument("--save", default=None, help="结束时保存分屏截图路径")
    ap.add_argument("--shot-dir", default="/mnt/d/captures/_locate/demo_shots")
    args = ap.parse_args()

    field = load_field(args.field)
    K, dist, _wh = load_intrinsics(args.intr)
    h, pitch, roll = load_extrinsics(args.extr)
    R_cb = camera_to_base(pitch, roll)
    map_names = [k for k in field if field_type(k) is not None]
    map_pts = np.array([field[k] for k in map_names], float)
    print("固定外参: z=%.3f m, pitch=%.2f°, roll=%.2f°（整段视频不变）"
          % (h, pitch, roll))
    model = get_model(args.weights)

    src = args.source
    if src.startswith("net://"):
        if NetCamera is None:
            sys.exit("net:// 源需要同目录的 net_camera.py")
        cap = NetCamera("http://" + src[6:])
    elif src.isdigit():
        cap = cv2.VideoCapture(int(src))
    else:
        cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        sys.exit(f"打不开视频源: {src}")

    view = TopView(field)
    hist = deque(maxlen=max(1, args.smooth))     # 最近若干帧的 (x, y, yaw)
    disp_xy, disp_yaw = None, None               # 平滑后显示值
    n_det, rms_last = 0, None
    valid, status = False, "INIT"
    fi = 0
    t0, n_proc = time.time(), 0
    os.makedirs(args.shot_dir, exist_ok=True)
    canvas = None

    # ---- 后台解算线程：显示循环不被 PnP 阻塞 ----
    lock = threading.Lock()
    job = {"busy": False, "fresh": False, "sols": None}
    worker_th = [None]

    def solve_worker(obs, types):
        # 固定外参链路：像素 → 地面载体系点 → 2D Kabsch 配准
        try:
            g, ok = pixels_to_ground(obs, K, dist, R_cb, h)
            if ok.sum() < 2:
                sols = []
            else:
                sols = solve_pose_fixed(g[ok], map_pts,
                                        [types[i] for i in np.where(ok)[0]],
                                        map_names)
        except Exception as e:
            print("solve error:", e)
            sols = []
        with lock:
            job["sols"], job["fresh"], job["busy"] = sols, True, False
        worker_th[0] = threading.current_thread()

    while True:
        ok, frame = cap.read()
        if not ok:
            if args.max_frames or args.no_show:  # 源读完即止（回放/冒烟）
                break
            print("视频源断开，0.5s 后重试...")
            time.sleep(0.5)
            continue
        fi += 1
        n_proc += 1
        if args.max_frames and fi > args.max_frames:
            break

        boxes = detect_boxes(model, frame, args.conf, args.imgsz,
                             args.enhance)
        n_det = len(boxes)

        # ---- 收取后台解算结果 ----
        with lock:
            fresh, sols = job["fresh"], job["sols"]
            job["fresh"] = False
        if fresh:
            if sols:
                s = sols[0]                  # 内点数最多、残差最小的解
                hist.append((s["x"], s["y"], s["yaw"]))
                rms_last = s["rms"]
            else:
                rms_last = None
        # ---- 派发新解算（每 stride 帧一次，检测每帧都做）----
        if boxes and not job["busy"] and fi % max(1, args.stride) == 0:
            with lock:
                job["busy"] = True
            obs = np.array([[b[1], b[2]] for b in boxes], float)
            types = [b[3] for b in boxes]
            th = threading.Thread(target=solve_worker, args=(obs, types),
                                  daemon=True)
            worker_th[0] = th
            th.start()
        if hist:
            arr = np.array(hist)
            disp_xy = (float(np.median(arr[:, 0])),
                       float(np.median(arr[:, 1])))
            disp_yaw = float(np.median(arr[:, 2]))
            view.trail.append(disp_xy)

        # ---- 状态机（细则要求的 valid / status 字段）----
        if n_det == 0:
            valid, status = False, "INVALID: no landmark detected"
        elif rms_last is None:
            valid, status = False, "INVALID: no consistent solution"
        elif rms_last <= 0.30:
            valid, status = True, "OK"
        else:
            valid, status = False, "SEARCHING: high residual"

        # ---- 渲染 ----
        W, H = 1600, 720
        canvas = np.zeros((H, W, 3), np.uint8)
        vw, vh = 960, 540                          # 左上视频区（等比缩放）
        small = cv2.resize(frame, (vw, vh))
        k = vw / frame.shape[1]
        for c, x, y, t, col, x1, y1, x2, y2 in boxes:
            p1 = (int(x1 * k), int(y1 * k))
            p2 = (int(x2 * k), int(y2 * k))
            cv2.rectangle(small, p1, p2, col, 2)
            cv2.putText(small, f"{t} {c:.2f}", (p1[0], max(14, p1[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        canvas[0:vh, 0:vw] = small

        fps = n_proc / max(time.time() - t0, 1e-6)
        res_s = "-" if rms_last is None else f"{rms_last:.3f} m"
        xy_s = "--.--- m, --.--- m" if disp_xy is None else \
            f"x = {disp_xy[0]:+.3f} m    y = {disp_xy[1]:+.3f} m"
        yaw_s = "---.-- deg" if disp_yaw is None else \
            f"yaw = {disp_yaw:+.2f} deg"
        now_s = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        hud = [
            f"weikai | {now_s} | task2 realtime localization "
            f"(fixed extr, map v2)  {fps:4.1f} fps",
            f"{xy_s}    {yaw_s}",
            f"z(height) = {h:.3f} m (fixed)    "
            f"pitch = {pitch:.1f} deg (fixed)    roll = {roll:.1f} deg",
            f"marker_count = {n_det}    residual = {res_s}",
            f"valid = {valid}    status = {status}",
        ]
        yy = vh + 34
        for i, s in enumerate(hud):
            col = (230, 230, 230)
            if i == 4:
                col = (0, 220, 0) if valid else (0, 0, 230)
            cv2.putText(canvas, s, (24, yy), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, col, 1, cv2.LINE_AA)
            yy += 32

        view.draw(canvas, vw, 0, cam=disp_xy, yaw=disp_yaw, valid=valid)

        if not args.no_show:
            live = src.startswith("net://") or src.isdigit()
            cv2.imshow("task2 realtime localization (q quit / s shot)",
                       canvas)
            key = cv2.waitKey(1 if live else 30) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                p = os.path.join(args.shot_dir,
                                 time.strftime("shot_%H%M%S.png"))
                cv2.imwrite(p, canvas)
                print("shot ->", p)

    if not args.no_show:
        cv2.destroyAllWindows()
    if worker_th[0] is not None and worker_th[0].is_alive():
        worker_th[0].join(timeout=60)            # 等 PnP 线程退出再卸载 OpenCV
    if args.save and canvas is not None:
        cv2.imwrite(args.save, canvas)
        print("saved", args.save)
    cap.release()


if __name__ == "__main__":
    main()
