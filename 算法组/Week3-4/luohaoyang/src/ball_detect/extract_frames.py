import argparse
from pathlib import Path

import cv2


def extract_one(video_path: Path, output_dir: Path, every: int, blur_threshold: float):
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"[跳过] 打不开 {video_path}")
        return 0, 0

    stem = video_path.stem
    frame_index = 0
    saved = 0
    skipped_blur = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        if frame_index % every == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()

            if sharpness < blur_threshold:
                skipped_blur += 1
            else:
                name = f"{stem}_{frame_index:06d}.jpg"
                path = output_dir / name
                cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved += 1

        frame_index += 1

    cap.release()
    print(
        f"{video_path.name}: 总帧数={frame_index}, 保存={saved}, "
        f"模糊跳过={skipped_blur}"
    )
    return saved, skipped_blur


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--videos", type=str, default="raw_videos",
                        help="视频目录或单个视频")
    parser.add_argument("--output", type=str, default="dataset/images/all",
                        help="抽帧输出目录")
    parser.add_argument("--every", type=int, default=10,
                        help="每隔多少帧取一张")
    parser.add_argument("--blur-threshold", type=float, default=40.0,
                        help="拉普拉斯方差低于该值视为模糊")
    args = parser.parse_args()

    video_input = Path(args.videos)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    if video_input.is_file():
        videos = [video_input]
    else:
        videos = sorted(
            list(video_input.glob("*.mp4"))
            + list(video_input.glob("*.avi"))
            + list(video_input.glob("*.mov"))
        )

    if not videos:
        raise SystemExit(f"没有在 {video_input} 找到视频")

    total = 0
    for video in videos:
        saved, _ = extract_one(video, output_dir, args.every, args.blur_threshold)
        total += saved

    print(f"全部完成，共保存 {total} 张到 {output_dir}")