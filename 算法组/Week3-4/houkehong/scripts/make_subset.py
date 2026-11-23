# -*- coding: utf-8 -*-
"""
从大数据集里挑"有代表性"的子集，用于人工标注

为什么需要：
    标志数据抽了 2400 张，但细则只要求 >=300 张有效标注图。
    全标一遍要好几小时，而且相邻帧高度重复，标了也不增值。
    所以按片段均匀取样 + 去重，压到 400 张左右。

为什么要保留片段结构：
    后面 train.py 按**片段**划分 train/val/test，所以子集必须保留
    <片段>/<图片> 的结构，不能拍平成一个目录。

用法：

    # 从 2400 张里挑约 400 张（9 段 x 45 张）
    python scripts/make_subset.py --src dataset_marker --dst dataset_marker_label --per-seg 45

    # 想要更少/更多就改 --per-seg
    python scripts/make_subset.py --src dataset_marker --dst dataset_marker_label --per-seg 30
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from src.config import FIELD_MARKER_CLASSES  # noqa: E402

# 现场实际存在的标志类别（统一配置在 src/config.py）
MARKER_ORDER = FIELD_MARKER_CLASSES


def pick_segment(files, n, min_diff=1.5):
    """
    在一段视频的帧里均匀挑 n 张，并跳过与上一张太像的。

    均匀取样（而不是随机）保证覆盖整段视频的视角变化范围。
    """
    total = len(files)
    if total <= n:
        return list(files), 0

    picked = []
    skipped_dup = 0
    prev_small = None
    # 均匀步长取样
    for i in range(n):
        idx = int(round(i * (total - 1) / max(1, n - 1)))
        p = files[idx]
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        small = cv2.resize(img, (160, 90), interpolation=cv2.INTER_AREA).astype(np.int16)
        if prev_small is not None:
            diff = float(np.mean(np.abs(small - prev_small)))
            if diff < min_diff:
                skipped_dup += 1
                # 与上一张太像，往后顺延找一个不一样的
                for k in range(idx + 1, min(idx + 6, total)):
                    img2 = cv2.imread(files[k], cv2.IMREAD_GRAYSCALE)
                    if img2 is None:
                        continue
                    s2 = cv2.resize(img2, (160, 90), interpolation=cv2.INTER_AREA).astype(np.int16)
                    if float(np.mean(np.abs(s2 - prev_small))) >= min_diff:
                        p = files[k]
                        small = s2
                        break
                else:
                    continue
        picked.append(p)
        prev_small = small
    return picked, skipped_dup


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(ROOT, "dataset_marker"))
    ap.add_argument("--dst", default=os.path.join(ROOT, "dataset_marker_label"))
    ap.add_argument("--per-seg", type=int, default=45, help="每段挑几张")
    ap.add_argument("--min-diff", type=float, default=1.5, help="去重阈值")
    ap.add_argument("--exclude", default="",
                    help="逗号分隔的片段名关键词，命中的整个片段排除")
    ap.add_argument("--move", action="store_true",
                    help="用移动而不是复制（大数据集想省空间时用）")
    args = ap.parse_args()

    segs = sorted([d for d in glob.glob(os.path.join(args.src, "*")) if os.path.isdir(d)])
    segs = [d for d in segs if glob.glob(os.path.join(d, "*.jpg"))]
    if not segs:
        raise SystemExit(f"{args.src} 下没有片段文件夹")

    # 排除指定片段。典型用途：排除"只拍到中圈圆弧"的片段，
    # 那里没有有效 L/T 标志，模型会对着圆弧交叉乱出框，全是误检。
    if args.exclude:
        keys = [k.strip() for k in args.exclude.split(",") if k.strip()]
        before = len(segs)
        segs = [d for d in segs if not any(k in os.path.basename(d) for k in keys)]
        print(f"[--exclude] 排除 {before - len(segs)} 个片段: {keys}")
        if not segs:
            raise SystemExit("排除后没有剩余片段")

    if os.path.isdir(args.dst):
        shutil.rmtree(args.dst)

    total_pick = 0
    print(f"{'片段':<26}{'原图':>7}{'选中':>7}")
    print("-" * 42)
    for d in segs:
        files = sorted(glob.glob(os.path.join(d, "*.jpg")))
        picked, dup = pick_segment(files, args.per_seg, args.min_diff)
        outdir = os.path.join(args.dst, os.path.basename(d))
        os.makedirs(outdir, exist_ok=True)
        for p in picked:
            dst = os.path.join(outdir, os.path.basename(p))
            if args.move:
                shutil.move(p, dst)
            else:
                shutil.copy2(p, dst)
        total_pick += len(picked)
        print(f"{os.path.basename(d):<26}{len(files):>7}{len(picked):>7}")

    print("-" * 42)
    print(f"{'合计':<26}{'':>7}{total_pick:>7}")
    print(f"\n输出: {args.dst}")
    print(f"片段数: {len(segs)}   图片: {total_pick}")
    if total_pick < 300:
        print("[注意] 细则要求 >= 300 张有效标注图，建议加大 --per-seg")
    print()
    print("下一步：标注")
    print(f"  .\\.venv\\Scripts\\python.exe scripts\\label_tool.py "
          f"--src {os.path.relpath(args.dst, ROOT)} --task marker")
    print("\n类别键（按现场实际类别）:", "  ".join(f"{i+1}={n}" for i, n in enumerate(MARKER_ORDER)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
