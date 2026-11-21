#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时足球检测演示（录屏用）。"""
from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


def open_camera(index: int, width: int, height: int) -> cv2.VideoCapture:
    if sys.platform.startswith("win"):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def main() -> int:
    ap = argparse.ArgumentParser(description="实时足球检测")
    ap.add_argument("--model", default="runs/detect/train/weights/best.pt")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.4, help="置信度阈值")
    ap.add_argument("--record", default="", help="录制输出文件名，如 demo.mp4")
    args = ap.parse_args()

    if not Path(args.model).exists():
        print(f"找不到模型：{args.model}")
        return 1

    model = YOLO(args.model)
    cap = open_camera(args.camera, args.width, args.height)
    if not cap.isOpened():
        print(f"打不开相机 index={args.camera}")
        return 1

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"相机 {w}x{h}，模型 {args.model}，conf={args.conf}")
    print("Q 退出 | S 截图 | R 开始/停止录制")

    writer = None
    fps_hist = deque(maxlen=30)
    infer_hist = deque(maxlen=30)
    frame_id = 0
    t_prev = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("读不到画面")
            break
        frame_id += 1
        now = time.time()
        dt = now - t_prev
        t_prev = now
        if dt > 0:
            fps_hist.append(1.0 / dt)

        t0 = time.time()
        results = model.predict(frame, imgsz=args.imgsz, conf=args.conf,
                                verbose=False, device=0 if _cuda() else "cpu")
        infer_ms = (time.time() - t0) * 1000
        infer_hist.append(infer_ms)

        view = frame.copy()
        det_count = 0
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                name = model.names.get(cls, str(cls))
                u, v = (x1 + x2) / 2, (y1 + y2) / 2
                det_count += 1

                cv2.rectangle(view, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(view, (int(u), int(v)), 4, (0, 0, 255), -1)
                label = f"{name} {conf:.2f} ({u:.0f},{v:.0f})"
                cv2.putText(view, label, (int(x1), max(20, int(y1) - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        fps = np.mean(fps_hist) if fps_hist else 0.0
        infer = np.mean(infer_hist) if infer_hist else 0.0
        bar = np.zeros((78, view.shape[1], 3), np.uint8)
        cv2.putText(bar, f"frame={frame_id}  det={det_count}  conf>={args.conf}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(bar, f"FPS={fps:5.1f}  infer={infer:5.1f} ms  {w}x{h}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        view = np.vstack([bar, view])

        if writer is not None:
            writer.write(view)
            cv2.putText(view, "REC", (view.shape[1] - 90, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("realtime ball detect", view)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            break
        elif key == ord("s"):
            name = f"frame_{frame_id:05d}.jpg"
            cv2.imwrite(name, view)
            print(f"截图：{name}")
        elif key == ord("r"):
            if writer is None:
                out = args.record or f"demo_{int(time.time())}.mp4"
                writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"),
                                         25.0, (view.shape[1], view.shape[0]))
                print(f"开始录制：{out}")
            else:
                writer.release()
                writer = None
                print("录制结束")

    if writer is not None:
        writer.release()
    cap.release()
    cv2.destroyAllWindows()
    print(f"共处理 {frame_id} 帧")
    return 0


def _cuda() -> bool:
    try:
        import torch
        return torch.cuda.is_available()
    except Exception:
        return False


if __name__ == "__main__":
    raise SystemExit(main())
