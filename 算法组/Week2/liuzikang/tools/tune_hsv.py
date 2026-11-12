#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""帮你给自己的照片"定 HSV 阈值"（换了自己的蓝色杯子照片时用这个）。

为什么需要它
------------
    默认阈值 (90,20,25)~(130,255,255) 是按我自己拍的照片标定的。
    你自己的照片如果偏暗/偏亮/色调不同，直接套可能检测不到。
    与其瞎试，不如让程序告诉你"图里的蓝色到底在哪个 H 上"。

做法
----
    1. 只统计"有颜色"的像素（S>80 且 V>40），避免灰墙、白纸干扰；
    2. 统计这些像素的 H 直方图，找出最主要的色相峰；
    3. 在峰的 ±10 范围内，统计 S、V 的分位数，给出推荐的上下界。

用法（在项目根目录）：
    python3 tools/tune_hsv.py data/images/photos/my_cup.jpg
    python3 tools/tune_hsv.py data/images/photos/*.jpg      # 一次看多张
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import imread_bgr, setup_stdout  # noqa: E402

import cv2  # noqa: E402


def analyse(path: Path) -> None:
    bgr = imread_bgr(path)
    print(f"\n===== {path.name} =====")
    if bgr is None:
        print("  [x] 读不到这张图")
        return

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    colored = (s > 80) & (v > 40)
    n = int(colored.sum())
    print(f"  尺寸 {bgr.shape[1]}x{bgr.shape[0]}   有颜色的像素 {n} 个 "
          f"({100.0 * n / (bgr.shape[0] * bgr.shape[1]):.1f}%)")
    if n < 100:
        print("  [!] 有颜色的像素太少，这张图可能整体偏灰/偏暗")
        return

    hist = np.bincount(h.astype(np.int32)[colored].ravel(), minlength=180)
    top = np.argsort(hist)[::-1][:6]
    print("  H 直方图前 6 名（H 0~180；蓝色大约 100~130）：")
    for hue in top:
        if hist[hue] == 0:
            continue
        bar = "#" * max(1, int(40 * hist[hue] / hist[top[0]]))
        print(f"    H={hue:3d}  {hist[hue]:8d}  {bar}")

    peak = int(top[0])
    lo_h, hi_h = max(0, peak - 10), min(180, peak + 10)
    sel = colored & (h >= lo_h) & (h <= hi_h)
    if int(sel.sum()) < 50:
        print("  [!] 峰附近像素太少，建议换一张背景更干净的照片")
        return

    s_sel, v_sel = s[sel], v[sel]
    s_lo = int(np.percentile(s_sel, 15))
    v_lo = int(np.percentile(v_sel, 10))
    print(f"\n  推荐阈值（直接复制去改 run_batch.py 的默认值）：")
    print(f"    --hsv-lower {lo_h} {s_lo} {v_lo}")
    print(f"    --hsv-upper {hi_h} 255 255")
    print(f"  该范围内的 S: 中位数 {int(np.median(s_sel))}, "
          f"V: 中位数 {int(np.median(v_sel))}")


def main() -> int:
    setup_stdout()
    args = sys.argv[1:]
    if not args:
        default = Path("data/images/photos")
        files = sorted(p for p in default.glob("*") if p.suffix.lower() in
                       (".jpg", ".jpeg", ".png", ".bmp")) if default.is_dir() else []
        if not files:
            print("用法：python3 tools/tune_hsv.py <图片路径> [更多图片...]")
            print("      也可以把自己的照片放进 data/images/photos/ 后直接运行本命令")
            return 1
        args = [str(f) for f in files]

    for a in args:
        p = Path(a)
        if not p.is_file():
            print(f"[x] 不是文件：{a}")
            continue
        analyse(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
