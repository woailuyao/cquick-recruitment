# -*- coding: utf-8 -*-
"""
场地标志自动预标注（L / T / X）

核心思路：
    细则对三类的定义，本质就是"交叉点有几条臂"：
        L 型 = 两条线成直角收尾       -> 2 条臂
        T 型 = 横线 + 竖线垂直顶上来  -> 3 条臂
        X 型 = 两条线交叉             -> 4 条臂
    所以不需要训练模型，用经典的"白色线段检测 + 交点 + 臂数统计"就能给出候选框，
    人工只需要删掉误检、补上漏检。

适用：PenaltyPoint（点球点）不适用——它是圆点，不是线交点。
     如果需要点球点，得用别的方法或手工标。

用法：
    python scripts/autolabel_markers.py --src dataset_marker_label
    python scripts/autolabel_markers.py --src dataset_marker_label --preview 20

输出：
    每张图旁生成同名 .txt（YOLO 格式），并写 <src>/autolabel_report.csv
    预览图存 snapshots/autolabel_preview/
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

CLASS_L = 0      # LCross
CLASS_T = 1      # TCross
CLASS_X = 2      # XCross
CLASS_NAMES = {CLASS_L: "LCross", CLASS_T: "TCross", CLASS_X: "XCross"}
CLASS_COLORS = {CLASS_L: (0, 165, 255), CLASS_T: (255, 170, 0), CLASS_X: (0, 255, 170)}


def white_mask(img):
    """提取白色场地线。草地是深绿，线是亮白，HSV 低饱和高亮最稳。"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (0, 0, 110), (180, 80, 255))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    m2 = cv2.inRange(gray, 165, 255)
    mask = cv2.bitwise_or(m, m2)
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    return mask


def line_intersect(a, b):
    """两条线段（HoughLinesP 结果）的交点；平行或交点不在两条线段范围内返回 None。"""
    x1, y1, x2, y2 = a
    x3, y3, x4, y4 = b
    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(d) < 1e-6:
        return None
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d

    def within(px, py, x1, y1, x2, y2, tol=12):
        lo_x, hi_x = min(x1, x2) - tol, max(x1, x2) + tol
        lo_y, hi_y = min(y1, y2) - tol, max(y1, y2) + tol
        return lo_x <= px <= hi_x and lo_y <= py <= hi_y

    if within(px, py, x1, y1, x2, y2) and within(px, py, x3, y3, x4, y4):
        return px, py
    return None


def count_arms(mask, cx, cy, radius=42, n_dir=32, min_run=0.45):
    """
    从交点出发沿 n_dir 个方向走，统计有几条"臂"。

    一条臂会让连续若干个方向都命中白色，所以要把相邻命中方向合并成一段，
    段的数量就是臂的数量。
    """
    hits = []
    for i in range(n_dir):
        ang = 2 * math.pi * i / n_dir
        hit = 0
        steps = max(6, int(radius))
        for r in range(4, steps + 1):
            x = int(round(cx + r * math.cos(ang)))
            y = int(round(cy + r * math.sin(ang)))
            if x < 0 or y < 0 or y >= mask.shape[0] or x >= mask.shape[1]:
                break
            if mask[y, x]:
                hit += 1
        hits.append(hit >= steps * min_run)

    # 环形合并连续命中
    if all(hits) or not any(hits):
        return 0
    n = len(hits)
    # 从任意一个非命中处开始，避免环形断开问题
    start = hits.index(False)
    runs = 0
    prev = False
    for k in range(n):
        cur = hits[(start + k) % n]
        if cur and not prev:
            runs += 1
        prev = cur
    return runs


def detect_markers(img):
    h, w = img.shape[:2]
    mask = white_mask(img)

    edges = cv2.Canny(mask, 50, 150)
    min_len = int(min(h, w) * 0.10)
    segs = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=45,
                           minLineLength=min_len, maxLineGap=12)
    if segs is None:
        return [], mask
    segs = np.asarray(segs).reshape(-1, 4)

    # 找所有线段交点
    pts = []
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            a, b = segs[i], segs[j]
            ang_a = math.degrees(math.atan2(a[3] - a[1], a[2] - a[0])) % 180
            ang_b = math.degrees(math.atan2(b[3] - b[1], b[2] - b[0])) % 180
            if min(abs(ang_a - ang_b), 180 - abs(ang_a - ang_b)) < 25:
                continue      # 近平行，交点不稳定
            p = line_intersect(a, b)
            if p is None:
                continue
            if not (30 <= p[0] < w - 30 and 20 <= p[1] < h - 20):
                continue
            pts.append(p)

    if not pts:
        return [], mask

    # 聚类：把 20 px 内的交点合并
    pts = np.array(pts, dtype=np.float64)
    used = np.zeros(len(pts), bool)
    clusters = []
    for i in range(len(pts)):
        if used[i]:
            continue
        d = np.linalg.norm(pts - pts[i], axis=1)
        grp = (d < 22) & (~used)
        used |= grp
        clusters.append(pts[grp].mean(axis=0))

    out = []
    for cx, cy in clusters:
        arms = count_arms(mask, cx, cy)
        if arms == 2:
            cls, size = CLASS_L, 46
        elif arms == 3:
            cls, size = CLASS_T, 50
        elif arms == 4:
            cls, size = CLASS_X, 52
        else:
            continue
        x1 = max(0, cx - size / 2); x2 = min(w - 1, cx + size / 2)
        y1 = max(0, cy - size / 2); y2 = min(h - 1, cy + size / 2)
        if (x2 - x1) < 12 or (y2 - y1) < 12:
            continue
        out.append((cls, x1, y1, x2, y2, arms))
    return out, mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(ROOT, "dataset_marker_label"))
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--preview", type=int, default=24, help="生成多少张预览图")
    args = ap.parse_args()

    imgs = sorted(glob.glob(os.path.join(args.src, "*", "*.jpg")))
    if not imgs:
        raise SystemExit(f"{args.src} 下没有图片")

    prev_dir = os.path.join(ROOT, "snapshots", "autolabel_preview")
    os.makedirs(prev_dir, exist_ok=True)
    budget = args.preview

    rows = []
    n_ok = 0
    cls_count = {CLASS_L: 0, CLASS_T: 0, CLASS_X: 0}

    print(f"自动预标注 {len(imgs)} 张 ...")
    for i, p in enumerate(imgs, 1):
        txt = os.path.splitext(p)[0] + ".txt"
        if os.path.exists(txt) and os.path.getsize(txt) > 0 and not args.overwrite:
            continue
        img = cv2.imread(p)
        if img is None:
            continue
        h, w = img.shape[:2]
        dets, mask = detect_markers(img)

        lines = []
        for cls, x1, y1, x2, y2, arms in dets:
            cx = (x1 + x2) / 2 / w
            cy = (y1 + y2) / 2 / h
            bw = (x2 - x1) / w
            bh = (y2 - y1) / h
            lines.append(f"{cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
            cls_count[cls] += 1
        with open(txt, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        if lines:
            n_ok += 1
        rows.append({"file": os.path.relpath(p, args.src), "markers": len(lines),
                     "types": ",".join(CLASS_NAMES.get(d[0], "?") for d in dets)})

        if budget > 0 and dets:
            vis = img.copy()
            for cls, x1, y1, x2, y2, arms in dets:
                col = CLASS_COLORS[cls]
                cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), col, 3)
                cv2.putText(vis, f"{CLASS_NAMES[cls]} arms={arms}",
                            (int(x1), max(18, int(y1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
            cv2.imwrite(os.path.join(prev_dir, os.path.basename(os.path.dirname(p))
                                     + "_" + os.path.basename(p)), vis)
            budget -= 1
        if i % 50 == 0:
            print(f"  {i}/{len(imgs)} ...")

    rep = os.path.join(args.src, "autolabel_report.csv")
    if rows:
        with open(rep, "w", newline="", encoding="utf-8-sig") as f:
            wtr = csv.DictWriter(f, fieldnames=["file", "markers", "types"])
            wtr.writeheader()
            wtr.writerows(rows)

    print("\n" + "=" * 60)
    print(f"有检出 {n_ok}/{len(rows)} 张")
    for c, name in CLASS_NAMES.items():
        print(f"  {name:<14} {cls_count[c]} 个")
    print(f"报告: {rep}")
    print(f"预览: {prev_dir}")
    print("\n⚠️ 自动结果**必须人工复核**：")
    print("   · 点球点不在自动检测范围内（它是圆点，不是线交点）")
    print("   · 中圈处的直-曲线交会被判成别的类别或漏掉")
    print("   · 远处的标志可能检测不到")
    print(f"  .\\.venv\\Scripts\\python.exe scripts\\label_tool.py "
          f"--src {os.path.relpath(args.src, ROOT)} --task marker")
    return 0


if __name__ == "__main__":
    sys.exit(main())
