import argparse
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=1)
    parser.add_argument("--model", default=str(ROOT / "runs" / "ball_v1" / "weights" / "best.pt"))
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"找不到模型: {model_path}")

    model = YOLO(str(model_path))

    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"打不开摄像头 index={args.camera}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 30)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    camera_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"camera index={args.camera}, {width}x{height}, {camera_fps:.1f} fps")
    print("press q or ESC to quit")

    frame_no = 0
    fps = 0.0
    last = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("read frame failed")
            break

        frame_no += 1

        t0 = time.perf_counter()
        results = model.predict(
            source=frame,
            conf=args.conf,
            imgsz=args.imgsz,
            device=args.device,
            verbose=False,
        )
        infer_ms = (time.perf_counter() - t0) * 1000

        result = results[0]
        if result.boxes is not None:
            for box in result.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                name = model.names.get(cls, str(cls))
                u = (x1 + x2) // 2
                v = (y1 + y2) // 2

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
                label = f"{name} {conf:.2f} center=({u},{v})"
                cv2.putText(frame, label, (x1, max(24, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        now = time.time()
        dt = max(1e-6, now - last)
        last = now
        inst_fps = 1.0 / dt
        fps = inst_fps if fps == 0 else 0.9 * fps + 0.1 * inst_fps

        cv2.putText(frame, f"frame={frame_no}  FPS={fps:.1f}  infer={infer_ms:.1f}ms",
                    (10, height - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(frame, time.strftime("%Y-%m-%d %H:%M:%S"),
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if frame_no % 15 == 0:
            print(f"frame={frame_no} fps={fps:.1f} infer={infer_ms:.1f}ms "
                  f"detections={len(result.boxes) if result.boxes is not None else 0}")

        cv2.imshow("football detection", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()