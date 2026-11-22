#!/usr/bin/env python3
"""任务1：足球实时检测。

加载 best.pt，接入相机实时画面，输出：
  - 类别名 ball、置信度、检测框 (x1,y1,x2,y2)
  - 检测框中心像素 (center_u, center_v)
  - 帧编号/时间戳、实时 FPS

用法：
    python detect_ball.py --weights best.pt --device /dev/video0 --conf 0.5
"""

import argparse
import time

import cv2
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="best.pt")
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    model = YOLO(args.weights)

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print("[FAIL] 无法打开相机")
        return 1

    print("开始实时足球检测，按 q 退出")
    frame_id = 0
    fps = 0.0
    t0 = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[FAIL] 取帧失败")
            break
        frame_id += 1

        results = model(frame, conf=args.conf, imgsz=640,verbose=False)
        boxes = results[0].boxes
        centers = []
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            name = results[0].names[cls]
            cu = (x1 + x2) / 2.0
            cv_ = (y1 + y2) / 2.0
            centers.append((cu, cv_))

            # 终端输出（结构化）
            print(
                f"[frame {frame_id}] {name} conf={conf:.2f} "
                f"bbox=({x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}) "
                f"center=({cu:.1f},{cv_:.1f})"
            )

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"{name} {conf:.2f}", (int(x1), int(y1) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.circle(frame, (int(cu), int(cv_)), 3, (0, 0, 255), -1)

        # FPS 统计（滑动窗口）
        if frame_id % 30 == 0:
            dt = time.time() - t0
            fps = 30 / dt
            t0 = time.time()
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        cv2.imshow("ball detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
