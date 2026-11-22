# -*- coding: utf-8 -*-
"""场地标志预标注：用白线几何自动生成 L / T / X 的候选框。

思路（不用任何神经网络）：
    1. 亮度/saturation 阈值提取白色线条，Canny 出边；
    2. HoughLinesP 得到线段；
    3. 两两求交点，角度接近垂直 / 斜交的都留下；
    4. 交点附近聚类，一个交点一个候选；
    5. 数「从这个交点向外的分支数」定类别：2 -> L，3 -> T，4 -> X；
    6. 按分支长度估框大小，写成 YOLO 格式（class cx cy w h，归一化）。

点球点不是线交点，本工具不生成，需人工补。

用法（项目根目录）：
    python tools/prelabel_marker.py --images dataset/raw_marker \
        --preview outputs/qc_marker/preview
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.common.paths import ROOT, rel                     # noqa: E402

CLASSES = ["LCross", "TCross", "XCross", "PenaltyPoint"]
IMG_EXTS = {".jpg", ".jpeg", ".png"}


# ---------------------------------------------------------------- 白线掩码
def white_mask(bgr: np.ndarray) -> np.ndarray:
    """提取「细白线」。关键是剔掉大块白色（墙面、窗、白布）——它们的角会被误判成 L 角。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    v, s = hsv[..., 2], hsv[..., 1]
    m = ((v > 165) & (s < 75)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    # 用大于线宽的开运算找出「厚块」，再从掩码里减掉 -> 只剩细长线条
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
    thick = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    thin = cv2.subtract(m, thick)
    thin = cv2.morphologyEx(thin, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(thin, 8)
    keep = np.zeros_like(thin)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= 30:
            keep[lab == i] = 255
    return keep


def line_width(mask: np.ndarray) -> float:
    """估白线宽度。用低分位数而不是中位数——画面里残留的大块白色会把中位数拉爆。"""
    dt = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
    vals = dt[dt > 0]
    if not vals.size:
        return 12.0
    return float(np.clip(np.percentile(vals, 25) * 2.0, 4.0, 30.0))


# ---------------------------------------------------------------- 交点求解
def seg_intersect(p1, p2, p3, p4, extend=30.0):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(d) < 1e-9:
        return None
    a = x1 * y2 - y1 * x2
    b = x3 * y4 - y3 * x4
    x = (a * (x3 - x4) - (x1 - x2) * b) / d
    y = (a * (y3 - y4) - (y1 - y2) * b) / d
    # Hough 线段常在交点附近断开，因此按「到线段的距离」判定，允许外延 extend 像素，
    # 而不是要求交点落在两条线段的包围盒内（后者会把绝大多数真实交点筛掉）。
    if point_seg_dist((x, y), p1, p2) > extend:
        return None
    if point_seg_dist((x, y), p3, p4) > extend:
        return None
    return float(x), float(y)


def angle_diff(a1, a2):
    d = abs(a1 - a2) % 180.0
    return min(d, 180.0 - d)


def point_seg_dist(pt, a, b):
    px, py = pt
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def count_arms(pt, lines, tol_px):
    """数从这个交点向外伸出的分支方向数（每 25° 一档去重）。"""
    dirs = []
    px, py = pt
    for (x1, y1, x2, y2) in lines:
        if point_seg_dist(pt, (x1, y1), (x2, y2)) > tol_px:
            continue
        for (ex, ey) in ((x1, y1), (x2, y2)):
            dx, dy = ex - px, ey - py
            L = math.hypot(dx, dy)
            if L < tol_px * 1.5:
                continue
            dirs.append((math.degrees(math.atan2(dy, dx)) % 360.0, L))
    if not dirs:
        return 0, []
    dirs.sort()
    groups = [[dirs[0]]]
    for d in dirs[1:]:
        if (d[0] - groups[-1][-1][0]) <= 25.0:
            groups[-1].append(d)
        else:
            groups.append([d])
    # 首尾跨 0° 的情况合并
    if len(groups) > 1 and (360.0 - groups[-1][-1][0] + groups[0][0][0]) <= 25.0:
        groups[0] = groups[-1] + groups[0]
        groups.pop()
    arms = [max(g, key=lambda t: t[1]) for g in groups]
    return len(arms), [a[1] for a in arms]


def radial_reach(mask: np.ndarray, cx: float, cy: float, r_max: int):
    """从 (cx,cy) 向 360° 打射线，返回每个角度上白色连通能延伸多远（像素）。

    比「线段求交」稳：场地标线在屏幕上是细白条，从交点看出去的分支数就是类别，
    L=2 支、T=3 支、X=4 支，不需要先把长直线拟合出来。
    """
    angles = np.arange(0.0, 360.0, 2.0)
    reach = np.zeros(len(angles), dtype=np.float32)
    h, w = mask.shape[:2]
    for i, a in enumerate(angles):
        ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
        gap = 0
        last = 0.0
        for r in range(2, r_max + 1):
            x = int(round(cx + ca * r))
            y = int(round(cy + sa * r))
            if x < 0 or y < 0 or x >= w or y >= h:
                break
            if mask[y, x] > 0:
                gap = 0
                last = float(r)
            else:
                gap += 1
                if gap > 3:          # 允许 3 px 的小缺口（线内有噪点）
                    break
        reach[i] = last
    return angles, reach


def classify_by_branches(mask: np.ndarray, dt: np.ndarray, cx: float, cy: float,
                         width: float, r_max: int,
                         l_angle=(70.0, 110.0), min_span: float = 8.0):
    """返回 (类别下标 或 None, 分支长度中位) —— 按分支数与分支夹角分类。"""
    angles, reach = radial_reach(mask, cx, cy, r_max)
    thr = max(width * 2.0, 7.0)
    on = reach > thr
    if not on.any():
        return None, 0.0

    # 找连续分支（角度环绕处要接起来）
    idx = np.nonzero(on)[0]
    runs = []
    start = idx[0]
    prev = idx[0]
    for k in idx[1:]:
        if k == prev + 1:
            prev = k
            continue
        runs.append((start, prev))
        start = prev = k
    runs.append((start, prev))
    if len(runs) > 1 and runs[0][0] == 0 and runs[-1][1] == len(angles) - 1:
        s, e = runs.pop()
        runs[0] = (s - len(angles), runs[0][1])

    step = 2.0
    arms = []
    h, w = mask.shape[:2]
    for s, e in runs:
        span = (e - s + 1) * step
        if span < min_span:          # 太窄不算分支，多半是噪点
            continue
        mid = int(((s + e) / 2) % len(angles))
        a = math.radians(mid * step)
        r_lo = max(int(0.55 * reach[mid]), 2)
        r_hi = max(int(reach[mid]) - 1, r_lo + 1)
        thick = []
        for r in range(r_lo, r_hi + 1, max((r_hi - r_lo) // 5, 1)):
            x = int(round(cx + math.cos(a) * r))
            y = int(round(cy + math.sin(a) * r))
            if 0 <= x < w and 0 <= y < h:
                thick.append(float(dt[y, x]))
        # 分支本身必须「细」，否则是大块白色（墙/布）的边角，不是地面标线
        if thick and float(np.median(thick)) > width * 1.25:
            return None, float(reach[mid])
        arms.append((mid * step, float(reach[mid])))
    if not arms:
        return None, 0.0

    k = len(arms)
    arm_len = float(np.median([a[1] for a in arms]))
    if k == 4:
        return 2, arm_len                     # XCross
    if k == 3:
        return 1, arm_len                     # TCross
    if k == 2:
        d = abs(arms[0][0] - arms[1][0]) % 360.0
        d = min(d, 360.0 - d)
        if l_angle[0] <= d <= l_angle[1]:     # 场地角；透视下直角在图上会变形，窗口可放宽
            return 0, arm_len                 # LCross
    return None, arm_len


def detect(bgr: np.ndarray, cfg: dict):
    h, w = bgr.shape[:2]
    mask = white_mask(bgr)
    dt = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
    width = line_width(mask)
    r_max = int(np.clip(width * 7.0, 26, 80))

    pts = cv2.goodFeaturesToTrack(mask, maxCorners=cfg["max_corners"],
                                  qualityLevel=cfg["quality"],
                                  minDistance=cfg["min_dist"], blockSize=9)
    if pts is None:
        return []
    cand = [(float(p[0][0]), float(p[0][1])) for p in pts]

    out = []
    used = []
    for (px, py) in cand:
        if not (8 <= px < w - 8 and 8 <= py < h - 8):
            continue
        if mask[int(py), int(px)] == 0:          # 交点本身必须是白的
            continue
        cls, arm_len = classify_by_branches(mask, dt, px, py, width, r_max)
        if cls is None:
            continue
        # 同一交点可能被多个角点重复命中，去重
        if any(math.hypot(px - ux, py - uy) < cfg["min_dist"] for ux, uy in used):
            continue
        used.append((px, py))
        half = float(np.clip(arm_len * 1.15, cfg["min_half"], cfg["max_half"]))
        x1 = float(np.clip(px - half, 0, w - 1))
        y1 = float(np.clip(py - half, 0, h - 1))
        x2 = float(np.clip(px + half, 0, w - 1))
        y2 = float(np.clip(py + half, 0, h - 1))
        if x2 - x1 < 16 or y2 - y1 < 16:
            continue
        out.append(dict(cls=cls, cx=px, cy=py, x1=x1, y1=y1, x2=x2, y2=y2,
                        n_arms=0, votes=1, width=width))
    return out


def default_cfg(size) -> dict:
    w = size[0]
    s = w / 1280.0
    return dict(vote=max(int(35 * s), 18), min_len=max(int(55 * s), 24),
                max_gap=max(int(10 * s), 5), cluster_px=max(int(16 * s), 8),
                min_votes=1, min_half=34.0, max_half=130.0,
                max_corners=120, quality=0.05, min_dist=max(int(26 * s), 12))


def main() -> int:
    ap = argparse.ArgumentParser(description="场地标志预标注（白线几何）")
    ap.add_argument("--images", default="dataset/raw_marker")
    ap.add_argument("--preview", default="outputs/qc_marker/preview")
    ap.add_argument("--limit", type=int, default=0, help="只处理前 N 张（调试用）")
    ap.add_argument("--skip-preview", action="store_true")
    args = ap.parse_args()

    idir = Path(args.images)
    if not idir.is_absolute():
        idir = ROOT / idir
    pdir = Path(args.preview)
    if not pdir.is_absolute():
        pdir = ROOT / pdir
    pdir.mkdir(parents=True, exist_ok=True)

    imgs = sorted(p for p in idir.rglob("*") if p.suffix.lower() in IMG_EXTS)
    if args.limit:
        imgs = imgs[: args.limit]
    if not imgs:
        print(f"{rel(idir)} 下没有图片，先跑 tools/frame_extract.py")
        return 1

    print("=" * 78)
    print(f"标志预标注：{len(imgs)} 张｜输出 YOLO txt 到图片同目录｜预览 {rel(pdir)}")
    print("=" * 78)
    tot = {c: 0 for c in CLASSES}
    n_box = 0
    empty = 0
    for k, p in enumerate(imgs, 1):
        bgr = cv2.imread(str(p))
        if bgr is None:
            continue
        cfg = default_cfg(bgr.shape)
        dets = detect(bgr, cfg)
        h, w = bgr.shape[:2]
        lines = []
        for d in dets:
            cx = (d["x1"] + d["x2"]) / 2 / w
            cy = (d["y1"] + d["y2"]) / 2 / h
            bw = (d["x2"] - d["x1"]) / w
            bh = (d["y2"] - d["y1"]) / h
            lines.append(f"{d['cls']} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
            tot[CLASSES[d["cls"]]] += 1
        n_box += len(dets)
        if not dets:
            empty += 1
        txt = p.with_suffix(".txt")
        txt.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

        if not args.skip_preview and (k <= 12 or k % 40 == 0):
            vis = bgr.copy()
            for d in dets:
                col = {0: (0, 200, 255), 1: (0, 255, 0), 2: (255, 120, 0)}[d["cls"]]
                cv2.rectangle(vis, (int(d["x1"]), int(d["y1"])),
                              (int(d["x2"]), int(d["y2"])), col, 2)
                cv2.putText(vis, CLASSES[d["cls"]], (int(d["x1"]), int(d["y1"]) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            cv2.imwrite(str(pdir / f"{p.parent.name}_{p.stem}.jpg"), vis,
                        [cv2.IMWRITE_JPEG_QUALITY, 85])

    print(f"共生成候选框 {n_box} 个；无候选的图 {empty} 张")
    for c in CLASSES[:3]:
        print(f"  {c:<9}: {tot[c]}")
    print(f"  {CLASSES[3]:<9}: 工具不生成，需人工补")
    print("-" * 78)
    print("下一步：python tools/annotate.py --dir dataset/raw_marker  人工核对增删")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
