"""粗线标志检测（RANSAC 直线版）—— 近景、线很粗时用这个。

思路：
    1. 白线掩码（草坪暗、线亮，阈值很干净）
    2. 掩码点做 RANSAC 拟合，找出 1~3 条"粗线带"（内点容差 = 线宽的一半）
    3. 两条不平行线的交点就是候选标志点
    4. 沿两条线四个方向探测白色是否延续，数胳膊：4=X，3=T，2=L
"""

import math
import random

import cv2
import numpy as np


def line_mask(bgr, thr=150, open_size=5):
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    m = (g > thr).astype(np.uint8) * 255
    if open_size:
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((open_size, open_size), np.uint8))
    return m


def _band_halfwidth(mask):
    """用距离变换估计粗线半宽（中位数）。"""
    d = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    vals = d[mask > 0]
    if vals.size == 0:
        return 10.0
    return float(np.percentile(vals, 85))


def fit_dominant_lines(mask, max_lines=3, iters=140, seed=0):
    """RANSAC 找粗线：每条线用 (点 p, 方向 d, 半宽 w) 表示。"""
    ys, xs = np.nonzero(mask > 0)
    if len(xs) < 200:
        return []
    pts = np.stack([xs, ys], 1).astype(np.float64)
    if len(pts) > 3000:
        idx = np.random.default_rng(seed).choice(len(pts), 3000, replace=False)
        pts = pts[idx]

    w = _band_halfwidth(mask)
    tol = max(6.0, w * 1.1)
    rng = random.Random(seed)
    lines = []
    remaining = pts.copy()

    for _ in range(max_lines):
        if len(remaining) < 300:
            break
        best_inl = None
        for _ in range(iters):
            i, j = rng.randrange(len(remaining)), rng.randrange(len(remaining))
            if i == j:
                continue
            p1, p2 = remaining[i], remaining[j]
            d = p2 - p1
            n = np.linalg.norm(d)
            if n < 200:  # 太短的点对不采用
                continue
            d /= n
            normal = np.array([-d[1], d[0]])
            dist = np.abs((remaining - p1) @ normal)
            inl = dist <= tol
            cnt = int(inl.sum())
            if best_inl is None or cnt > best_inl[0]:
                best_inl = (cnt, inl, d, p1)
        if best_inl is None or best_inl[0] < len(remaining) * 0.12:
            break

        cnt, inl, d, p1 = best_inl
        # 用全部内点重新拟合（最小二乘）
        Q = remaining[inl]
        mean = Q.mean(0)
        u, s, vt = np.linalg.svd(Q - mean)
        d = vt[0] / np.linalg.norm(vt[0])
        lines.append({"p": mean, "d": d, "w": w, "n": int(cnt)})
        remaining = remaining[~inl]
        if len(lines) >= max_lines:
            break
    return lines


def _intersect(l1, l2):
    p1, d1 = l1["p"], l1["d"]
    p2, d2 = l2["p"], l2["d"]
    A = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]])
    if abs(np.linalg.det(A)) < 1e-9:
        return None
    t, u = np.linalg.solve(A, p2 - p1)
    return p1 + t * d1


def _arm(mask, p, d, start, length, tol):
    """沿 d 方向，从 start 到 length 距离，白色是否连续。"""
    h, w = mask.shape[:2]
    hit = tot = 0
    for t in np.linspace(start, length, 6):
        q = p + d * t
        x, y = int(round(q[0])), int(round(q[1]))
        if not (0 <= x < w and 0 <= y < h):
            continue
        tot += 1
        x0, x1 = max(0, x - tol), min(w, x + tol + 1)
        y0, y1 = max(0, y - tol), min(h, y + tol + 1)
        if mask[y0:y1, x0:x1].max() > 0:
            hit += 1
    return tot > 0 and hit / tot >= 0.75


def detect(bgr, want=("L", "T", "X"), max_lines=3, scale=None):
    """scale>1 时先把图缩小再检测（快很多），坐标自动换算回原图。"""
    small = bgr
    k = 1.0
    if scale is None:
        scale = 2.0 if bgr.shape[1] >= 1000 else 1.0
    if scale > 1.0:
        k = float(scale)
        small = cv2.resize(bgr, None, fx=1.0 / k, fy=1.0 / k, interpolation=cv2.INTER_AREA)
    dets = _detect_impl(small, want=want, max_lines=max_lines)
    if k != 1.0:
        for d in dets:
            d["u"] *= k
            d["v"] *= k
            d["w"] *= k
    return dets


def _detect_impl(bgr, want=("L", "T", "X"), max_lines=3):
    mask = line_mask(bgr)
    lines = fit_dominant_lines(mask, max_lines=max_lines)
    if len(lines) < 2:
        return []
    h, w = mask.shape[:2]
    out = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            a1 = abs(math.degrees(math.atan2(lines[i]["d"][1], lines[i]["d"][0])))
            a2 = abs(math.degrees(math.atan2(lines[j]["d"][1], lines[j]["d"][0])))
            da = abs(a1 - a2) % 180
            da = min(da, 180 - da)
            if da < 25:
                continue
            p = _intersect(lines[i], lines[j])
            if p is None or not (10 <= p[0] < w - 10 and 10 <= p[1] < h - 10):
                continue
            wmax = max(lines[i]["w"], lines[j]["w"])
            probe = max(60.0, 2.2 * wmax)
            tol = max(3, int(wmax * 0.5))
            arms = 0
            for ln in (lines[i], lines[j]):
                for sgn in (1.0, -1.0):
                    if _arm(mask, p, ln["d"] * sgn, max(8.0, wmax * 0.8), probe, tol):
                        arms += 1
            typ = {4: "X", 3: "T", 2: "L"}.get(arms)
            if typ is None or typ not in want:
                continue
            out.append({"type": typ, "u": float(p[0]), "v": float(p[1]),
                        "conf": min(1.0, 0.45 + 0.13 * arms), "arms": arms,
                        "w": float(wmax)})
    out.sort(key=lambda c: (-{"X": 3, "T": 2, "L": 1}[c["type"]], -c["conf"]))
    kept = []
    for c in out:
        if all(math.hypot(c["u"] - k["u"], c["v"] - k["v"]) > 110 for k in kept):
            kept.append(c)
    return kept


def draw(bgr, dets):
    vis = bgr.copy()
    palette = {"L": (0, 200, 255), "T": (80, 220, 80), "X": (255, 140, 60)}
    for d in dets:
        c = palette.get(d["type"], (255, 255, 255))
        p = (int(round(d["u"])), int(round(d["v"])))
        cv2.circle(vis, p, 18, c, 3, cv2.LINE_AA)
        cv2.drawMarker(vis, p, c, cv2.MARKER_CROSS, 34, 2)
        cv2.putText(vis, "%s %.2f" % (d["type"], d["conf"]), (p[0] + 20, p[1] - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, c, 2, cv2.LINE_AA)
    return vis
