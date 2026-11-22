import argparse
import time
from pathlib import Path

import cv2


def record(camera_index: int, output: Path, seconds: int, width: int, height: int, fps: float):
    output.parent.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"打不开摄像头 index={camera_index}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    ok, frame = cap.read()
    if not ok:
        cap.release()
        raise SystemExit("摄像头打开成功，但读不到画面")

    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = cap.get(cv2.CAP_PROP_FPS) or fps
    print(f"实际参数: {real_w}x{real_h}, {real_fps:.1f} fps")
    print(f"输出文件: {output}")
    print("按 q 可提前停止。")

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output), fourcc, real_fps, (real_w, real_h))
    if not writer.isOpened():
        cap.release()
        raise SystemExit("VideoWriter 打开失败，请换一个输出文件名或安装编码器")

    start = time.time()
    frames = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("读帧失败，停止。")
            break

        elapsed = time.time() - start
        remain = max(0.0, seconds - elapsed)

        cv2.putText(
            frame,
            f"REC {remain:5.1f}s  frame={frames}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2,
        )
        cv2.imshow("record - press q to stop", frame)
        writer.write(frame)
        frames += 1

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or elapsed >= seconds:
            break

    cap.release()
    writer.release()
    cv2.destroyAllWindows()

    duration = time.time() - start
    print(f"完成: {frames} 帧, {duration:.1f} 秒, 保存到 {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=1,
                        help="摄像头序号，0 通常是内置，1 通常是 USB")
    parser.add_argument("--output", type=str, required=True,
                        help="例如 raw_videos/video_01.mp4")
    parser.add_argument("--seconds", type=int, default=30)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    record(
        camera_index=args.camera,
        output=Path(args.output),
        seconds=args.seconds,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )