# -*- coding: utf-8 -*-
"""任务二机位对照预览：实时画面 + 标志检测框 + 检出计数。

用途：现场挪相机时对着这个窗口调，直到稳定出现 ≥4 个标志（解外参的最低门槛，
且要求分布铺开、交点在画面内）。窗口左上角大字号显示当前检出数：
    NEED 未满足 -> 红色   OK -> 绿色
并显示"已连续达标多少秒"，避免抖动误判。

按键：q 或 ESC 退出；s 存当前帧到 outputs/realtime/。

用法（项目根目录）：
    .\\run.bat tools\\marker_preview.py
    .\\run.bat tools\\marker_preview.py --conf 0.25 --scale 0.6
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import OUTPUT_DIR, WEIGHTS_DIR, rel          # noqa: E402

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                                  # noqa: BLE001
    pass

W, H = 1280, 720
NEED = 3          # 与 matcher.min_markers 一致（定位硬门槛）
PREFER = 4        # 稳妥值，凑到更好
COLORS = {0: (255, 160, 0), 1: (0, 200, 255), 2: (0, 255, 0)}      # L / T / X


def open_camera(index: int):
    for name, api in (("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)):
        cap = cv2.VideoCapture(index, api)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
            cap.set(cv2.CAP_PROP_FPS, 30)
            return cap, name
        cap.release()
    return None, ""


def main() -> int:
    ap = argparse.ArgumentParser(description="任务二机位对照预览")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--weights", default=str(WEIGHTS_DIR / "best_marker_v9.pt"))
    ap.add_argument("--conf", type=float, default=0.30,
                    help="检测阈值；0.73 m 低机位掠射视角下要降到 0.12~0.15 才出框")
    ap.add_argument("--dedup", type=float, default=35.0,
                    help="同点去重半径（像素）：同一交点被重复判类时只留最高分")
    ap.add_argument("--device", default="0")
    ap.add_argument("--scale", type=float, default=0.72, help="窗口缩放（默认 0.72）")
    ap.add_argument("--imgsz", type=int, default=960,
                    help="推理分辨率；低机位标志小，960/1280 比 640 召回高（默认 960）")
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)

    cap, backend = open_camera(args.index)
    if cap is None:
        print(f"打不开相机 index={args.index}（可能被其他程序占用）")
        return 1

    win = "marker preview (q to quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, int(W * args.scale), int(H * args.scale))
    cv2.moveWindow(win, 60, 40)

    # 预热：首帧建 CUDA 上下文
    for _ in range(5):
        cap.read()

    ok_since = None
    t_prev = time.perf_counter()
    fps_win: list[float] = []

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("取帧失败")
            break

        res = model.predict(frame, conf=args.conf, imgsz=args.imgsz, device=args.device,
                            verbose=False)[0]
        raw = []
        if res.boxes is not None and len(res.boxes) > 0:
            raw = list(zip(res.boxes.xyxy.cpu().numpy(),
                           res.boxes.conf.cpu().numpy(),
                           res.boxes.cls.cpu().numpy()))
        # 同点去重：低阈值（0.73 m 掠射视角只能给到 0.1~0.2）下，同一个交点常被
        # 同时判成 T/L/X 三个框，位置几乎重合。按像素距离合并、留最高分再计数。
        kept, centers = [], []
        for xyxy, cf, cls in sorted(raw, key=lambda z: -float(z[1])):
            cu = (float(xyxy[0]) + float(xyxy[2])) / 2.0
            cv_ = (float(xyxy[1]) + float(xyxy[3])) / 2.0
            if any(((cu - px) ** 2 + (cv_ - py) ** 2) ** 0.5 <= args.dedup
                   for px, py in centers):
                continue
            centers.append((cu, cv_))
            kept.append((xyxy, cf, cls))

        n = len(kept)
        labels = []
        for xyxy, cf, cls in kept:
            k = int(cls)
            name = model.names[k] if isinstance(model.names, dict) else model.names[k]
            labels.append(f"{name[:1]}{float(cf):.2f}")
            x1, y1, x2, y2 = [int(v) for v in xyxy]
            color = COLORS.get(k, (0, 255, 0))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            cv2.putText(frame, f"{name} {float(cf):.2f}", (x1, max(y1 - 8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)

        now = time.perf_counter()
        fps_win.append(1.0 / max(now - t_prev, 1e-9))
        t_prev = now
        fps = sum(fps_win[-20:]) / len(fps_win[-20:])

        if n >= NEED:
            ok_since = ok_since if ok_since is not None else now
            held = now - ok_since
        else:
            ok_since = None
            held = 0.0

        good = n >= NEED
        # 半透明黑底，避免白墙上看不清
        cv2.rectangle(frame, (0, 0), (720, 92), (0, 0, 0), -1)
        cv2.putText(frame, f"markers {n} / need >= {NEED} (prefer {PREFER}+)   "
                           f"{'OK' if good else 'NOT YET'}",
                    (12, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 0) if good else (0, 0, 255), 3)
        cv2.putText(frame, f"hold {held:4.1f}s   fps {fps:4.1f}   conf>={args.conf}   "
                           f"imgsz={args.imgsz}   cam idx={args.index} {backend}   {' '.join(labels)}",
                    (12, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255) if good else (200, 200, 200), 1)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
            p = OUTPUT_DIR / f"preview_{time.strftime('%H%M%S')}.png"
            cv2.imwrite(str(p), frame)
            print(f"已存图 {rel(p)}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
