"""从原始视频抽帧，并过滤掉严重模糊和几乎重复的帧。

用法：
    python extract_frames.py                     # 每 8 帧取 1 帧
    python extract_frames.py --step 5            # 抽密一点
    python extract_frames.py --keep-blur         # 不丢模糊帧

输出：data/frames/<clip>/<clip>_000123.jpg
标注时直接在这个目录上标，标签 .txt 就写在图片旁边（同名）。
"""
import argparse
import csv
import glob
import os

import cv2
import numpy as np

IMG_EXT = (".avi", ".mp4", ".mov", ".mkv")


def small_gray(frame):
    """缩到 128x72 的灰度图，用来判断两帧是不是几乎一样。"""
    return cv2.cvtColor(cv2.resize(frame, (128, 72)), cv2.COLOR_BGR2GRAY).astype(np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default="data/raw")
    ap.add_argument("--out", default="data/frames")
    ap.add_argument("--step", type=int, default=8, help="每隔 N 帧取一张")
    ap.add_argument("--blur", type=float, default=60.0, help="拉普拉斯方差低于此值视为模糊")
    ap.add_argument("--diff", type=float, default=4.0, help="与上一张保留帧的平均灰度差低于此值视为重复")
    ap.add_argument("--keep-blur", action="store_true", help="保留模糊帧")
    args = ap.parse_args()

    clips = sorted(p for p in glob.glob(os.path.join(args.raw, "*")) if p.lower().endswith(IMG_EXT))
    if not clips:
        print(f"{args.raw} 下没有视频（.avi/.mp4）。先用 capture.py 采集。")
        return 1

    os.makedirs(args.out, exist_ok=True)
    rows = []
    total_kept = 0
    for clip_path in clips:
        clip = os.path.splitext(os.path.basename(clip_path))[0]
        save_dir = os.path.join(args.out, clip)
        os.makedirs(save_dir, exist_ok=True)

        cap = cv2.VideoCapture(clip_path)
        if not cap.isOpened():
            print(f"[x] 打不开 {clip_path}")
            continue
        src_fps = cap.get(cv2.CAP_PROP_FPS) or 0
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)

        kept = dropped_blur = dropped_dup = 0
        idx = 0
        prev = None
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if idx % args.step == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
                cur_small = small_gray(frame)

                if not args.keep_blur and sharp < args.blur:
                    dropped_blur += 1
                elif prev is not None and float(np.mean(np.abs(cur_small - prev))) < args.diff:
                    dropped_dup += 1
                else:
                    name = f"{clip}_{idx:06d}.jpg"
                    cv2.imwrite(os.path.join(save_dir, name), frame,
                                [cv2.IMWRITE_JPEG_QUALITY, 95])
                    prev = cur_small
                    kept += 1
            idx += 1
        cap.release()

        total_kept += kept
        print(f"{clip:>12}  源 {idx} 帧 @{src_fps:.0f}fps  取 {kept} 张"
              f"（模糊丢 {dropped_blur}，重复丢 {dropped_dup}）")
        rows.append([clip, idx, src_fps, kept, dropped_blur, dropped_dup])

    print("-" * 66)
    print(f"合计保留 {total_kept} 张，来源 {len(rows)} 段视频。")
    if total_kept < 200:
        print(f"还差 {200 - total_kept} 张，建议再采 1~2 段（或用 --step 4 抽密一点）。")

    os.makedirs("outputs", exist_ok=True)
    with open("outputs/frames_summary.csv", "w", newline="", encoding="utf-8-sig") as f:
        wr = csv.writer(f)
        wr.writerow(["clip", "source_frames", "source_fps", "kept", "dropped_blur", "dropped_dup"])
        wr.writerows(rows)
    print("统计已写入 outputs/frames_summary.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
