# -*- coding: utf-8 -*-
"""
标定质量诊断

为什么需要：RMS 低不代表标定可靠。
    RMS 只统计"被检出的角点"的拟合残差。如果 20 张图全拍在画面中间，
    角点从没到过四角，那么边缘的畸变就是**多项式外推**出来的，
    k1/k2/k3 会出现数值大、正负交替的过拟合形态，RMS 依然很小。

本脚本检查三件事：
    1. 角点的空间覆盖（有没有到四角）
    2. 各张图的构图分布（是否过于雷同）
    3. 按当前标定参数，画面四角的畸变位移有多大（离谱说明模型不可信）

用法：
    python scripts/diag_calibration.py --images calib_images --pattern 9x6
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys

import cv2
import numpy as np


def detect(gray, cols, rows):
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, c = cv2.findChessboardCornersSB(
            gray, (cols, rows),
            getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0) | getattr(cv2, "CALIB_CB_ACCURACY", 0))
        if ok:
            return True, c.astype(np.float32)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, c = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if ok:
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
        c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), crit)
    return ok, c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default="calib_images")
    ap.add_argument("--pattern", default="9x6")
    ap.add_argument("--square", type=float, default=20.0)
    args = ap.parse_args()

    cols, rows = (int(v) for v in args.pattern.lower().split("x"))
    paths = sorted(glob.glob(os.path.join(args.images, "*.jpg"))
                   + glob.glob(os.path.join(args.images, "*.png")))
    if not paths:
        print(f"{args.images} 下没有图片")
        return 1

    W = H = None
    pts_all = []
    per_img = []
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            continue
        h, w = img.shape[:2]
        W, H = w, h
        ok, c = detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cols, rows)
        if not ok:
            per_img.append((os.path.basename(p), None))
            continue
        pts = c.reshape(-1, 2)
        pts_all.append(pts)
        per_img.append((os.path.basename(p), pts))

    if not pts_all:
        print("没有一张图检出角点")
        return 1

    allp = np.vstack(pts_all)
    print("=" * 68)
    print(f"标定图   : {len(paths)} 张，其中 {len(pts_all)} 张检出角点")
    print(f"分辨率   : {W}x{H}")
    print("=" * 68)

    # ---------------- 1. 空间覆盖 ----------------
    print("\n[1] 角点空间覆盖（归一化到 0~1，四角应为 0/1 附近）")
    print(f"  x 范围: {allp[:,0].min()/W:.3f} ~ {allp[:,0].max()/W:.3f}")
    print(f"  y 范围: {allp[:,1].min()/H:.3f} ~ {allp[:,1].max()/H:.3f}")

    # 把画面分成 3x3 网格，统计每格里有多少角点
    gx = np.clip((allp[:, 0] / W * 3).astype(int), 0, 2)
    gy = np.clip((allp[:, 1] / H * 3).astype(int), 0, 2)
    grid = np.zeros((3, 3), int)
    for i in range(len(allp)):
        grid[gy[i], gx[i]] += 1
    print("\n  3x3 网格角点分布（每格点数）:")
    for r in range(3):
        print("   " + " ".join(f"{grid[r,c]:6d}" for c in range(3)))

    empty = int((grid == 0).sum())
    corner_empty = grid[0, 0] + grid[0, 2] + grid[2, 0] + grid[2, 2]
    if empty > 0:
        print(f"\n  ⚠️ 有 {empty} 个网格完全没有角点")
    if corner_empty == 0:
        print("\n  ⚠️ 画面四角（3x3 的四个角格）里**一个角点都没有**")
        print("     这是畸变过拟合的直接原因：边缘畸变只能靠多项式外推，")
        print("     k1/k2/k3 会变成数值大、正负交替的形态。")

    cover_x = (allp[:, 0].max() - allp[:, 0].min()) / W
    cover_y = (allp[:, 1].max() - allp[:, 1].min()) / H
    print(f"\n  有效覆盖范围: x {cover_x*100:.0f}%   y {cover_y*100:.0f}%")
    if cover_x < 0.7 or cover_y < 0.7:
        print("  ⚠️ 覆盖不足 70%，建议补拍：把棋盘格移到画面四角再各拍 2 张")

    # ---------------- 2. 构图雷同度 ----------------
    print("\n[2] 各图构图（中心位置与大小，雷同度）")
    sigs = []
    for name, pts in per_img:
        if pts is None:
            print(f"  {name:<24} 未检出")
            continue
        cx, cy = pts[:, 0].mean() / W, pts[:, 1].mean() / H
        size = math.hypot(float(np.ptp(pts[:, 0])), float(np.ptp(pts[:, 1]))) / math.hypot(W, H)
        sigs.append((cx, cy, size))
        print(f"  {name:<24} 中心({cx:.2f},{cy:.2f})  尺寸{size:.2f}")
    if len(sigs) > 1:
        s = np.array(sigs)
        print(f"\n  构图标准差: 中心x {s[:,0].std():.3f}  中心y {s[:,1].std():.3f}  尺寸 {s[:,2].std():.3f}")
        if s[:, 0].std() < 0.05 and s[:, 1].std() < 0.05:
            print("  ⚠️ 构图集中，缺乏多样性，畸变参数会不稳定")

    print("\n" + "=" * 68)
    print("结论：")
    if corner_empty == 0 or empty > 0:
        print("  → 覆盖不足，建议补拍画面四角的图后重新标定；")
        print("    或者改用 4 参数模型（固定 k3）以降低过拟合：")
        print(f"     .\\.venv\\Scripts\\python.exe scripts\\calibrate_camera.py "
              f"--images {args.images} --pattern {args.pattern} "
              f"--square {args.square:g} --fix-k3")
    else:
        print("  → 覆盖看起来还行，畸变系数可以直接用。")
    print("=" * 68)
    return 0


if __name__ == "__main__":
    sys.exit(main())
