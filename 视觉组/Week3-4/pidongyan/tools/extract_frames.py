"""从原始视频中抽帧，生成待标注图片池。

设计要点（对应考核要求）：
  * 文件名保留来源视频前缀：<视频名>_f<帧号>.jpg
    后续 split_dataset.py 靠这个前缀按「独立视频片段」划分数据集，防止相邻帧泄漏。
  * 支持清晰度过滤（拉普拉斯方差），剔除运动模糊废片。
  * 支持相邻帧去重，避免同一段视频抽出大量高度相似帧。
  * 打印每段视频的分辨率/标称帧率，分辨率不是 1280x720 时给出警告。

用法：
  python tools/extract_frames.py                       # 处理 raw_videos 下全部视频，每秒抽 2 帧
  python tools/extract_frames.py --fps 3
  python tools/extract_frames.py --videos raw_videos/ball_run01_20261110_101500.mp4
  python tools/extract_frames.py --min-sharpness 100 --dedup-thresh 2.0

抽帧建议：每段视频按 2~3 fps 抽，5 段 × 60 秒 ≈ 600~1000 帧，
        再从中挑出 200+ 张覆盖不同距离/位置/遮挡/光照的样本用于标注。
"""
import argparse
import csv
import datetime as dt
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_VIDEO_DIR = ROOT / "raw_videos"
DEFAULT_OUT = ROOT / "frames_pool"
LOG_CSV = ROOT / "logs" / "extract_log.csv"

VIDEO_EXTS = {".mp4", ".avi", ".mov", ".mkv", ".mpg", ".mpeg", ".wmv", ".m4v", ".flv"}


def gather_videos(paths):
    files = []
    for p in paths:
        p = Path(p)
        if p.is_dir():
            files.extend(sorted(f for f in p.iterdir() if f.suffix.lower() in VIDEO_EXTS))
        elif p.is_file() and p.suffix.lower() in VIDEO_EXTS:
            files.append(p)
        else:
            print(f"[跳过] 不是视频文件或目录：{p}")
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", nargs="*", default=[str(DEFAULT_VIDEO_DIR)],
                    help="视频文件或目录，默认 raw_videos/")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="抽帧输出目录")
    ap.add_argument("--fps", type=float, default=2.0, help="每秒抽帧数")
    ap.add_argument("--max-per-video", type=int, default=0, help="每段视频最多保留帧数，0=不限")
    ap.add_argument("--min-sharpness", type=float, default=0.0,
                    help="清晰度下限（拉普拉斯方差），0=不过滤；建议 50~150")
    ap.add_argument("--dedup-thresh", type=float, default=0.0,
                    help="相邻帧去重阈值（灰度平均绝对差，0-255），0=不去重；建议 1.5~3.0")
    args = ap.parse_args()

    videos = gather_videos(args.videos)
    if not videos:
        raise SystemExit(f"没有找到视频，请把原始视频放到 {DEFAULT_VIDEO_DIR}")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    LOG_CSV.parent.mkdir(parents=True, exist_ok=True)
    log_rows = []

    print(f"共 {len(videos)} 段视频，目标 {args.fps} fps\n")

    for video in videos:
        cap = cv2.VideoCapture(str(video))
        if not cap.isOpened():
            print(f"[错误] 打不开 {video.name}")
            continue

        src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        step = max(1, int(round(src_fps / args.fps)))

        flag = "" if (w, h) == (1280, 720) else "  [警告] 非 1280x720，与考核统一分辨率不一致"
        print(f"- {video.name}  {w}x{h}  {src_fps:.1f} fps  {total} 帧"
              f"  每 {step} 帧取 1{flag}")

        idx = 0
        kept = blur_skip = dup_skip = 0
        last_small = None

        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if idx % step == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
                small = cv2.resize(gray, (64, 36)).astype(np.float32)

                is_dup = (
                    args.dedup_thresh > 0
                    and last_small is not None
                    and float(np.mean(np.abs(small - last_small))) < args.dedup_thresh
                )

                if args.min_sharpness > 0 and sharp < args.min_sharpness:
                    blur_skip += 1
                elif is_dup:
                    dup_skip += 1
                else:
                    name = f"{video.stem}_f{idx:06d}.jpg"
                    cv2.imwrite(str(out_dir / name), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    kept += 1
                    last_small = small
                    if args.max_per_video and kept >= args.max_per_video:
                        break
            idx += 1

        cap.release()
        print(f"  保留 {kept} 张 | 模糊跳过 {blur_skip} | 重复跳过 {dup_skip}")
        log_rows.append([
            dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), video.name,
            f"{w}x{h}", f"{src_fps:.1f}", total, kept, blur_skip, dup_skip,
        ])

    new_file = not LOG_CSV.exists()
    with LOG_CSV.open("a", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow(["时间", "视频", "分辨率", "标称帧率", "总帧数",
                             "保留帧数", "模糊跳过", "重复跳过"])
        writer.writerows(log_rows)

    total_kept = sum(r[5] for r in log_rows)
    print(f"\n总计抽出 {total_kept} 张 -> {out_dir}")
    print(f"日志：{LOG_CSV}")
    if total_kept < 200:
        print("[提示] 不足 200 张：加长视频时长、提高 --fps，或补拍更多片段")


if __name__ == "__main__":
    main()
