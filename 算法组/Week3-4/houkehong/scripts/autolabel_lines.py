# -*- coding: utf-8 -*-
"""
基于直线检测的场地标志候选生成器（L / T / X）

为什么不用 YOLO 预标注
----------------------
标志模型在你这个机位完全失效（画面被压扁 6 倍，训练集里没有这种形态）。
所以换一条路：不依赖学习，直接利用场地线的几何特性。

思路
----
    1. 提取白色线掩膜（草地是深绿，线是高亮低饱和，很好分）
    2. Hough 变换找**长直线段**
    3. 把共线的线段合并成无限长直线
    4. 两两求交点，只保留落在画面内的
    5. **数臂**：对每个交点，看这两条直线各自向两侧是否还在延伸
           2 条臂 -> L 型
           3 条臂 -> T 型
           4 条臂 -> X 型
    6. 生成 YOLO 格式的候选框，人工只做增删改

数臂是关键：它直接对应细则对 L/T/X 的定义（两线成直角收尾 = L，横竖相交 = T，
两线交叉 = X），而且用的是直线本身的走向，不受透视变形影响。

用法
----
    # 在图片或整个目录上跑，输出预览图
    python scripts/autolabel_lines.py --src snapshots --pattern "shot_*_raw.jpg" --preview

    # 对标注目录生成 .txt 候选（不覆盖已有非空标签）
    python scripts/autolabel_lines.py --src dataset_marker_v2 --write
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

CLS_L, CLS_T, CLS_X = 0, 1, 2
NAMES = {CLS_L: "LCross", CLS_T: "TCross", CLS_X: "XCross"}
COLORS = {CLS_L: (0, 165, 255), CLS_T: (255, 170, 0), CLS_X: (0, 255, 170)}


def white_mask(img, v_min=150, s_max=70):
    """草坪上的白色线：高亮度 + 低饱和。"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (0, 0, v_min), (180, s_max, 255))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    m2 = cv2.inRange(gray, int(v_min * 1.05), 255)
    mask = cv2.bitwise_or(m1, m2)
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    return mask


def find_lines(mask, h, w, min_len_ratio=0.12, max_lines=60):
    """找长直线，返回 [(p0, p1), ...] 像素坐标。"""
    edges = cv2.Canny(mask, 50, 150)
    min_len = int(min(h, w) * min_len_ratio)
    segs = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50,
                           minLineLength=min_len, maxLineGap=18)
    if segs is None:
        return []
    segs = np.asarray(segs).reshape(-1, 4)

    # 按长度降序，合并近似共线的
    items = []
    for x1, y1, x2, y2 in segs:
        L = math.hypot(x2 - x1, y2 - y1)
        ang = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180
        items.append((L, ang, (x1, y1, x2, y2)))
    items.sort(key=lambda t: -t[0])

    kept = []
    for L, ang, seg in items:
        dup = False
        for _, ang2, seg2 in kept:
            dang = min(abs(ang - ang2), 180 - abs(ang - ang2))
            if dang > 4.0:
                continue
            # 垂距：seg 中点到 seg2 所在直线
            x1, y1, x2, y2 = seg2
            mx, my = (seg[0] + seg[2]) / 2, (seg[1] + seg[3]) / 2
            den = math.hypot(x2 - x1, y2 - y1)
            if den < 1e-6:
                continue
            d = abs((x2 - x1) * (y1 - my) - (x1 - mx) * (y2 - y1)) / den
            if d < 22:
                dup = True
                break
        if not dup:
            kept.append((L, ang, seg))
        if len(kept) >= max_lines:
            break
    return [seg for _, _, seg in kept]


def line_intersect(a, b, h, w, margin=8, tol=90):
    x1, y1, x2, y2 = a
    x3, y3, x4, y4 = b
    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(d) < 1e-6:
        return None
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d
    if not (margin <= px < w - margin and margin <= py < h - margin):
        return None
    # 交点必须落在线段附近（容差大一些，因为线段可能被截断）
    def near(px, py, x1, y1, x2, y2, tol=tol):
        lo_x, hi_x = min(x1, x2) - tol, max(x1, x2) + tol
        lo_y, hi_y = min(y1, y2) - tol, max(y1, y2) + tol
        return lo_x <= px <= hi_x and lo_y <= py <= hi_y
    if not (near(px, py, x1, y1, x2, y2) and near(px, py, x3, y3, x4, y4)):
        return None
    return px, py


def arm_present(mask, cx, cy, dx, dy, radius=55, ratio=0.55):
    """从交点沿 (dx,dy) 方向走，判断这段线是否还延续（够长、够白）。"""
    n = math.hypot(dx, dy)
    if n < 1e-6:
        return False
    dx, dy = dx / n, dy / n
    hits = 0
    total = max(8, int(radius))
    for r in range(6, total):
        x = int(round(cx + r * dx))
        y = int(round(cy + r * dy))
        if x < 0 or y < 0 or y >= mask.shape[0] or x >= mask.shape[1]:
            break
        # 允许 ±2 px 的横向容差（线宽 / 抗锯齿）
        found = False
        for o in (-2, -1, 0, 1, 2):
            xx = int(round(x - o * dy))
            yy = int(round(y + o * dx))
            if 0 <= yy < mask.shape[0] and 0 <= xx < mask.shape[1] and mask[yy, xx]:
                found = True
                break
        if found:
            hits += 1
    return hits >= (total - 6) * ratio


def count_arms(mask, cx, cy, lineA, lineB, radius=55):
    """
    数交点上有几条臂。

    对每条直线，分别看它向两个方向是否延伸。直线方向从线段取，
    所以即使透视把角的形状改变了，也能正确判断。
    """
    arms = 0
    for (x1, y1, x2, y2) in (lineA, lineB):
        dx, dy = x2 - x1, y2 - y1
        if arm_present(mask, cx, cy, dx, dy, radius):
            arms += 1
        if arm_present(mask, cx, cy, -dx, -dy, radius):
            arms += 1
    return arms


def detect(img, min_len_ratio=0.12, radius=55, box_size=None, tol=25):
    h, w = img.shape[:2]
    mask = white_mask(img)
    lines = find_lines(mask, h, w, min_len_ratio=min_len_ratio)
    if len(lines) < 2:
        return [], mask

    cand = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            a, b = lines[i], lines[j]
            ang_a = math.degrees(math.atan2(a[3] - a[1], a[2] - a[0])) % 180
            ang_b = math.degrees(math.atan2(b[3] - b[1], b[2] - b[0])) % 180
            if min(abs(ang_a - ang_b), 180 - abs(ang_a - ang_b)) < 20:
                continue
            p = line_intersect(a, b, h, w, tol=tol)
            if p is None:
                continue
            cand.append((p, a, b))

    # 合并邻近交点
    merged = []
    for p, a, b in cand:
        keep = True
        for m in merged:
            if math.hypot(p[0] - m[0][0], p[1] - m[0][1]) < 30:
                keep = False
                break
        if keep:
            merged.append((p, a, b))

    out = []
    for (cx, cy), a, b in merged:
        arms = count_arms(mask, cx, cy, a, b, radius)
        if arms == 2:
            cls = CLS_L
        elif arms == 3:
            cls = CLS_T
        elif arms == 4:
            cls = CLS_X
        else:
            continue
        s = box_size or 70
        x1 = max(0, cx - s / 2); y1 = max(0, cy - s / 2)
        x2 = min(w - 1, cx + s / 2); y2 = min(h - 1, cy + s / 2)
        out.append((cls, x1, y1, x2, y2, arms, int(cx), int(cy)))
    return out, mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(ROOT, "snapshots"))
    ap.add_argument("--pattern", default="shot_*_raw.jpg")
    ap.add_argument("--write", action="store_true", help="对目录生成 .txt（不覆盖已有非空标签）")
    ap.add_argument("--preview", action="store_true")
    ap.add_argument("--radius", type=int, default=55)
    ap.add_argument("--box", type=int, default=70)
    ap.add_argument("--min-len", type=float, default=0.12)
    ap.add_argument("--tol", type=int, default=25, help="交点必须落在线段端点多近处（像素）")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.src, args.pattern)))
    if not files:
        # 直接按目录找图
        files = sorted(glob.glob(os.path.join(args.src, "*.jpg")))
    if not files:
        print(f"{args.src} 下没有匹配 {args.pattern} 的图片")
        return 2

    prev_dir = os.path.join(ROOT, "snapshots", "line_prelabel")
    if args.preview:
        os.makedirs(prev_dir, exist_ok=True)

    n_tot = 0
    print(f"{'文件':<30}{'L':>4}{'T':>4}{'X':>4}{'合计':>6}   位置")
    print("-" * 78)
    for p in files:
        img = cv2.imread(p)
        if img is None:
            continue
        h, w = img.shape[:2]
        dets, mask = detect(img, args.min_len, args.radius,
                            box_size=args.box if args.box else None, tol=args.tol)
        cnt = {CLS_L: 0, CLS_T: 0, CLS_X: 0}
        pos = []
        for d in dets:
            cnt[d[0]] += 1
            pos.append(f"{NAMES[d[0]][0]}({d[6]},{d[7]})")
        n_tot += len(dets)
        print(f"{os.path.basename(p)[:28]:<30}{cnt[CLS_L]:>4}{cnt[CLS_T]:>4}{cnt[CLS_X]:>4}"
              f"{len(dets):>6}   {', '.join(pos[:6])}")

        if args.write:
            txt = os.path.splitext(p)[0] + ".txt"
            if os.path.isfile(txt) and os.path.getsize(txt) > 0:
                pass                     # 已有标注就不动
            else:
                lines = []
                for cls, x1, y1, x2, y2, arms, cx, cy in dets:
                    lines.append(f"{cls} {(x1+x2)/2/w:.6f} {(y1+y2)/2/h:.6f} "
                                 f"{(x2-x1)/w:.6f} {(y2-y1)/h:.6f}")
                with open(txt, "w", encoding="utf-8") as f:
                    f.write("\n".join(lines) + ("\n" if lines else ""))

        if args.preview:
            vis = img.copy()
            for cls, x1, y1, x2, y2, arms, cx, cy in dets:
                col = COLORS[cls]
                cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), col, 3)
                cv2.drawMarker(vis, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 14, 2)
                cv2.putText(vis, f"{NAMES[cls]} arms={arms}", (int(x1), max(16, int(y1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
            cv2.imwrite(os.path.join(prev_dir, os.path.basename(p)), vis)

    print("-" * 78)
    print(f"合计 {n_tot} 个候选，处理 {len(files)} 张图")
    if args.preview:
        print(f"预览 -> {prev_dir}")
    print("\n⚠️ 这是**候选**不是最终标签：")
    print("   · 数臂只对『两条直线相交』才准；圆弧、球门网、阴影边缘都可能混进来")
    print("   · 务必用 label_tool 人工过一遍：删误检、补漏检、改错类别")
    return 0


if __name__ == "__main__":
    sys.exit(main())
