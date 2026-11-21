from __future__ import annotations

import argparse
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="YOLOv8 单类别实时检测（摄像头）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--weights", required=True, help="训练得到的 best.pt")
    p.add_argument("--camera", default="0", help="摄像头编号，一般是 0")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--device", default="", help="留空自动；也可以写 cpu 或 0")
    p.add_argument("--save", default="", help="把带检测框的画面录成 mp4，例如 demo.mp4")
    p.add_argument("--line", type=float, default=2.0, help="框线宽")
    return p


def open_camera(source: str, width: int, height: int) -> cv2.VideoCapture:
    cam = int(source) if str(source).isdigit() else source
    cap = cv2.VideoCapture(cam)
    if not cap.isOpened():
        raise SystemExit(
            f"[错误] 打不开摄像头 {source}。\n"
            "排查：1) v4l2-ctl --list-devices 看设备节点 2) 换个编号试试 "
            "3) 有没有别的程序占着摄像头"
        )
    # MJPG 在多数 UVC 摄像头上才能稳定跑 1280x720@30
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def main() -> None:
    args = build_parser().parse_args()

    from ultralytics import YOLO
    model = YOLO(r"C:\Users\yangshuyuan\Desktop\soccer_dataset\runs\detect\runs\detect\ball_yolov8n\weights\best.pt")
    names = model.names

    cap = open_camera(args.camera, args.width, args.height)
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[信息] 摄像头已打开，实际分辨率 {real_w}x{real_h}")

    writer = None
    if args.save:
        out_path = Path(args.save).expanduser().resolve()
        writer = cv2.VideoWriter(
            str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (real_w, real_h)
        )
        if not writer.isOpened():
            raise SystemExit(f"[错误] 无法创建视频文件：{out_path}")
        print(f"[信息] 正在录制：{out_path}")

    fps_window: deque[float] = deque(maxlen=30)
    frame_id = 0
    t_start = time.perf_counter()

    print("[信息] 按 q 退出，按 s 截图。")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[警告] 读帧失败，摄像头可能被拔出或被占用。")
            break

        frame_id += 1
        t0 = time.perf_counter()

        results = model.predict(
            frame,
            imgsz=args.imgsz,
            conf=args.conf,
            device=args.device or None,
            verbose=False,
        )
        infer_ms = (time.perf_counter() - t0) * 1000.0

        boxes = results[0].boxes
        for box in boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)

            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            u = (x1 + x2) / 2.0
            v = (y1 + y2) / 2.0

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), int(args.line))
            cv2.circle(frame, (int(u), int(v)), 4, (0, 0, 255), -1)

            text = f"{label} {conf:.2f} ({u:.0f},{v:.0f})"
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            ty = max(y1 - 8, th + 4)
            cv2.rectangle(frame, (x1, ty - th - 4), (x1 + tw + 6, ty + 2), (0, 255, 0), -1)
            cv2.putText(frame, text, (x1 + 3, ty - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        now = time.perf_counter()
        fps_window.append(now)
        fps = 0.0
        if len(fps_window) >= 2:
            span = fps_window[-1] - fps_window[0]
            fps = (len(fps_window) - 1) / span if span > 0 else 0.0

        hud = [
            f"FPS {fps:5.1f} | infer {infer_ms:5.1f} ms | {real_w}x{real_h}",
            f"frame {frame_id} | det {len(boxes)} | conf>={args.conf}",
            f"t={now - t_start:7.1f}s",
        ]
        for i, line in enumerate(hud):
            cv2.putText(frame, line, (10, 26 + 26 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3)
            cv2.putText(frame, line, (10, 26 + 26 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 1)

        if writer is not None:
            writer.write(frame)

        cv2.imshow("YOLOv8 ball detect (q to quit)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            shot = f"snapshot_{frame_id:06d}.jpg"
            cv2.imwrite(shot, frame)
            print(f"[信息] 已保存截图：{shot}")

    cap.release()
    if writer is not None:
        writer.release()
        print(f"[信息] 录制结束：{args.save}")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
