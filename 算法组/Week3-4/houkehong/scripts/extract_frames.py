# -*- coding: utf-8 -*-
"""
从实拍视频抽帧（生成待标注数据集）

细则 1.3 / 2.4 的两个硬要求决定了这个工具不能只是"每秒存一帧"：

  1. "同一段视频连续抽取的高度相似帧不能被当作大量独立样本"
     -> 必须去重：和上一张保留帧差异太小的直接扔掉。

  2. "数据集按独立视频片段划分，不能先把一段视频的相邻帧随机打散到三个集合"
     -> 所以输出按**源视频分文件夹**，后面划分数据集也按文件夹整体分，
        不会把同一段视频的相邻帧泄漏到 train 和 val 两边。

另外还做了模糊剔除：Laplacian 方差太低的帧（运动模糊、对焦没拉到）标注价值很低，
留着只会污染训练集。

用法：

    # 从 recordings/ 里所有视频抽帧
    python scripts/extract_frames.py --src recordings --dst dataset_raw

    # 只抽某几段，控制密度
    python scripts/extract_frames.py --src recordings --dst dataset_raw \
        --every-n 3 --min-diff 2.5 --min-sharp 40

    # 抽完看统计
    python scripts/extract_frames.py --stats dataset_raw

输出：
    dataset_raw/
      ├── ball_20261108_161230/         # 一段视频一个文件夹
      │   ├── frame_000001.jpg
      │   └── ...
      ├── ball_20261108_161540/
      └── frames_manifest.csv           # 每帧来源/时间戳/清晰度/差异
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def sharpness(frame):
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(g, cv2.CV_64F).var())


def extract_one(video_path, out_root, every_n, min_diff, min_sharp, max_frames,
                skip_existing=False):
    name = os.path.splitext(os.path.basename(video_path))[0]
    outdir = os.path.join(out_root, name)

    # 补录场景专用：该片段已经抽过帧就整个跳过。
    # 理由：抽帧是"按顺序编号"的（frame_000001...），如果重新抽一遍，
    # 一旦去重参数有变化，编号就会移位，已有的标注 txt 全部对不上图。
    if skip_existing:
        old = glob.glob(os.path.join(outdir, "*.jpg"))
        if old:
            print(f"\n[{name}] 已存在 {len(old)} 张，跳过（--skip-existing）")
            return []

    os.makedirs(outdir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[跳过] 打不开 {video_path}")
        return []

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    print(f"\n[{name}]  {total} 帧 @ {fps:.1f} fps")

    kept = []
    prev_small = None
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % max(1, every_n) != 0:
            idx += 1
            continue

        # 去重：和上一张保留帧比，差异太小就丢
        small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
        if prev_small is not None:
            diff = float(np.mean(np.abs(small.astype(np.int16) - prev_small.astype(np.int16))))
            if diff < min_diff:
                idx += 1
                continue
        else:
            diff = 999.0

        sh = sharpness(frame)
        if sh < min_sharp:
            idx += 1
            continue

        p = os.path.join(outdir, f"frame_{len(kept) + 1:06d}.jpg")
        cv2.imwrite(p, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        kept.append({
            "segment": name,
            "file": os.path.basename(p),
            "source_video": os.path.basename(video_path),
            "source_frame": idx,
            "time_s": round(idx / fps, 3) if fps else "",
            "sharpness": round(sh, 1),
            "diff_vs_prev": round(diff, 2),
        })
        prev_small = small

        if max_frames and len(kept) >= max_frames:
            print(f"  达到上限 {max_frames} 张，停止")
            break
        idx += 1

    cap.release()
    print(f"  保留 {len(kept)} 张（源 {total} 帧，抽帧间隔 {every_n}，"
          f"差异阈值 {min_diff}，清晰度阈值 {min_sharp}）")
    return kept


def stats(dataset_dir):
    segs = [d for d in glob.glob(os.path.join(dataset_dir, "*")) if os.path.isdir(d)]
    if not segs:
        print(f"{dataset_dir} 下没有片段文件夹")
        return 1
    print("=" * 66)
    print(f"数据集统计: {dataset_dir}")
    print("=" * 66)
    print(f"{'片段':<34} {'图片数':>6}")
    print("-" * 66)
    total = 0
    for s in sorted(segs):
        n = len(glob.glob(os.path.join(s, "*.jpg")))
        total += n
        flag = "" if n >= 20 else "  <- 偏少"
        print(f"{os.path.basename(s):<34} {n:>6}{flag}")
    print("-" * 66)
    print(f"{'合计':<34} {total:>6}")
    print(f"片段数: {len(segs)}")
    if len(segs) < 5:
        print("[注意] 细则要求 >= 5 段独立视频")
    if total < 200:
        print("[注意] 足球数据要求 >= 200 张有效标注图；标志数据 >= 300 张")
    print("\n划分数据集时**按片段整体分配**，例如：")
    names = [os.path.basename(s) for s in sorted(segs)]
    n = len(names)
    if n < 5:
        print(f"  片段只有 {n} 段，不够分。至少再录 {5 - n} 段（细则要求 >= 5 段独立视频）。")
    else:
        n_test = max(1, round(n * 0.10))
        n_val = max(1, round(n * 0.20))
        n_train = n - n_test - n_val
        if n_train < 1:
            print(f"  片段只有 {n} 段，分不出 train/val/test，再多录几段。")
        else:
            print(f"  test  ({n_test} 段): {names[:n_test]}")
            print(f"  val   ({n_val} 段): {names[n_test:n_test + n_val]}")
            print(f"  train ({n_train} 段): {names[n_test + n_val:]}")
            print("  注意：同一段视频的所有帧必须整个落在同一个集合里，不能拆开。")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(ROOT, "recordings"), help="视频目录")
    ap.add_argument("--dst", default=os.path.join(ROOT, "dataset_raw"), help="输出目录")
    ap.add_argument("--every-n", type=int, default=2, help="每隔几帧取一帧")
    ap.add_argument("--min-diff", type=float, default=2.0,
                    help="与上一张保留帧的平均灰度差，低于此值视为重复帧丢弃")
    ap.add_argument("--min-sharp", type=float, default=25.0,
                    help="Laplacian 方差低于此值视为模糊帧丢弃")
    ap.add_argument("--max-per-seg", type=int, default=0, help="每段最多保留几张，0=不限")
    ap.add_argument("--pattern", default="*.mp4", help="视频文件名通配")
    ap.add_argument("--skip-existing", action="store_true",
                    help="该片段已抽过帧就跳过。补录新视频时**务必加上**，"
                         "否则重抽旧片段会让帧编号移位、已有标注对不上图")
    ap.add_argument("--stats", default=None, help="只统计已有数据集")
    args = ap.parse_args()

    if args.stats:
        return stats(args.stats)

    videos = sorted(glob.glob(os.path.join(args.src, args.pattern)))
    if not videos:
        print(f"[错误] {args.src} 下没有找到 {args.pattern}")
        return 1

    print(f"找到 {len(videos)} 段视频")
    os.makedirs(args.dst, exist_ok=True)

    all_rows = []
    for v in videos:
        all_rows += extract_one(v, args.dst, args.every_n, args.min_diff,
                                args.min_sharp, args.max_per_seg,
                                skip_existing=args.skip_existing)

    manifest = os.path.join(args.dst, "frames_manifest.csv")
    if all_rows:
        with open(manifest, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
            w.writeheader()
            w.writerows(all_rows)
        print(f"\n清单: {manifest}")

    print("\n" + "=" * 60)
    print(f"共保留 {len(all_rows)} 张")
    print("下一步：")
    print(f"  .\\.venv\\Scripts\\python.exe scripts\\prelabel.py --src {args.dst}")
    print("  或用 labelImg / CVAT 手动标，导出 YOLO 格式")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
