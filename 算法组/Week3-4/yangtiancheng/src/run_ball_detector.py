#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
from ultralytics import YOLO


def parse_source(value: str):
    return int(value) if value.isdigit() else value


parser = argparse.ArgumentParser(description="YOLO足球实时检测")
parser.add_argument(
    "--source",
    default="0",
    help="摄像头编号（默认0）或视频文件路径",
)
parser.add_argument(
    "--model",
    default=str(
        Path.home()
        / "vision_ws/k1_soccer_vision/runs/ball_detect/ball_v3_hardneg/weights/best.pt"
    ),
)
parser.add_argument("--conf", type=float, default=0.4)
parser.add_argument("--imgsz", type=int, default=640)
args = parser.parse_args()

model_path = Path(args.model).expanduser()
if not model_path.is_file():
    raise FileNotFoundError(f"模型不存在：{model_path}")

model = YOLO(str(model_path))
source = parse_source(args.source)
capture = cv2.VideoCapture(source)

if not capture.isOpened():
    raise RuntimeError(f"无法打开摄像头或视频：{args.source}")

miss_count = 0
last_box = None

print("检测已启动；按 q 退出")

while True:
    ok, frame = capture.read()
    if not ok:
        break

    result = model(
        frame,
        imgsz=args.imgsz,
        conf=args.conf,
        max_det=1,
        device="cpu",
        verbose=False,
    )[0]

    boxes = result.boxes
    held = False

    if len(boxes) > 0:
        x1, y1, x2, y2 = map(int, boxes.xyxy[0].cpu().tolist())
        confidence = float(boxes.conf[0])
        last_box = (x1, y1, x2, y2, confidence)
        miss_count = 0
    else:
        miss_count += 1
        # 容忍连续视频中1至2帧的短暂漏检。
        if last_box is not None and miss_count <= 2:
            x1, y1, x2, y2, confidence = last_box
            held = True
        else:
            last_box = None

    if last_box is not None:
        x1, y1, x2, y2, confidence = last_box
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        color = (0, 165, 255) if held else (0, 255, 0)
        status = "held" if held else "ball"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.circle(frame, (center_x, center_y), 4, color, -1)
        cv2.putText(
            frame,
            f"{status} {confidence:.2f} ({center_x},{center_y})",
            (x1, max(25, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )
    else:
        cv2.putText(
            frame,
            "ball not found",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )

    cv2.imshow("Ball detector - q to quit", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

capture.release()
cv2.destroyAllWindows()
