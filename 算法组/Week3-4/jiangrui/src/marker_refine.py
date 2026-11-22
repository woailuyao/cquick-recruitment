# -*- coding: utf-8 -*-
"""场地标志的几何细化：从 bbox 内提取真实交点像素，而不是直接用框中心。

规则:
    T / L / X : 在框内分割白色标线，Hough 提取两簇方向不同的线段，
                用总体最小二乘拟合两条直线，取交点作为 landmark_pixel。
    P(点球点) : 取白色圆形区域的形心。
    细化失败（框太小、线段不足、交点跑出框外）时退回 bbox 中心，
                并把 refined 标记为 False，方便后续降权或排查。

实测说明:
    场地标线是白色胶带，草地是深色，用自适应阈值分割比较稳。
    如果换到深色标线 / 浅色地面，可传 invert=True。
"""

import math

import cv2
import numpy as np


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _largest_component_near_center(mask, cx, cy):
    """保留与中心区域相交的面积最大连通域，去掉草叶、人影等碎块。"""
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return mask
    h, w = mask.shape
    best, best_area = -1, 0
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < 15:
            continue
        # 框中心是否落在该连通域（或非常接近）
        near = x - 3 <= cx <= x + bw + 3 and y - 3 <= cy <= y + bh + 3
        if near and area > best_area:
            best, best_area = i, area
    if best < 0:
        return mask
    return np.where(labels == best, 255, 0).astype(np.uint8)


def _fit_line(points):
    """总体最小二乘直线：返回 (点, 单位方向)。"""
    pts = np.asarray(points, dtype=float)
    center = pts.mean(axis=0)
    if len(pts) < 2:
        return center, np.array([1.0, 0.0])
    _, _, vt = np.linalg.svd(pts - center)
    direction = vt[0]
    direction = direction / (np.linalg.norm(direction) + 1e-9)
    if direction[0] < 0:
        direction = -direction
    return center, direction


def _intersect(p1, d1, p2, d2):
    a = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]], dtype=float)
    det = np.linalg.det(a)
    if abs(det) < 0.15:
        return None
    lam = np.linalg.solve(a, p2 - p1)
    # 交点是否合理交给外层的“交点须在框附近”检查
    return p1 + lam[0] * d1


def _angle_diff(a, b):
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def refine_pixel(frame, box, mtype, invert=False, margin=0.25, min_side=24):
    """返回 (u, v, refined, method)。u/v 为原图像素坐标。"""
    x1, y1, x2, y2 = [float(v) for v in box]
    H, W = frame.shape[:2]
    x1i, y1i = int(_clamp(math.floor(x1), 0, W - 1)), int(_clamp(math.floor(y1), 0, H - 1))
    x2i, y2i = int(_clamp(math.ceil(x2), 0, W - 1)), int(_clamp(math.ceil(y2), 0, H - 1))
    cx_box = (x1 + x2) / 2.0
    cy_box = (y1 + y2) / 2.0
    if x2i - x1i < 3 or y2i - y1i < 3:
        return cx_box, cy_box, False, "tiny"

    w, h = x2i - x1i, y2i - y1i
    if min(w, h) < min_side:
        return cx_box, cy_box, False, "small"

    pad = int(margin * max(w, h)) + 4
    rx1, ry1 = max(0, x1i - pad), max(0, y1i - pad)
    rx2, ry2 = min(W, x2i + pad), min(H, y2i + pad)
    roi = frame[ry1:ry2, rx1:rx2]
    if roi.size == 0:
        return cx_box, cy_box, False, "empty"

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if roi.ndim == 3 else roi
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    block = 31 if min(gray.shape[:2]) >= 40 else 15
    mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                 cv2.THRESH_BINARY, block, -8)
    if invert:
        mask = cv2.bitwise_not(mask)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    cx_local, cy_local = int(cx_box - rx1), int(cy_box - ry1)
    mask = _largest_component_near_center(mask, cx_local, cy_local)
    area = int(mask.sum() / 255)
    if area < 20:
        return cx_box, cy_box, False, "no_white"

    if mtype == "P":
        ys, xs = np.nonzero(mask)
        return float(rx1 + xs.mean()), float(ry1 + ys.mean()), True, "centroid"

    min_len = max(10, int(0.28 * min(w, h)))
    lines = cv2.HoughLinesP(mask, 1, np.pi / 180,
                            threshold=max(12, int(0.12 * min(w, h))),
                            minLineLength=min_len, maxLineGap=6)
    if lines is None or len(lines) < 2:
        return cx_box, cy_box, False, "no_line"

    segs = []
    # OpenCV 不同版本返回 (N,1,4) 或 (N,4)，统一摊平
    for ln in np.asarray(lines).reshape(-1, 4)[:120]:
        xa, ya, xb, yb = [float(v) for v in ln]
        length = math.hypot(xb - xa, yb - ya)
        if length < 4:
            continue
        ang = math.degrees(math.atan2(yb - ya, xb - xa)) % 180.0
        segs.append({"p1": np.array([xa, ya]), "p2": np.array([xb, yb]),
                     "len": length, "ang": ang})
    if len(segs) < 2:
        return cx_box, cy_box, False, "no_line"

    # 按方向聚成两簇（取总长度最大的两个）
    clusters = []
    for seg in sorted(segs, key=lambda s: -s["len"]):
        placed = False
        for c in clusters:
            if _angle_diff(seg["ang"], c["ang"]) < 18.0:
                c["segs"].append(seg)
                c["len"] += seg["len"]
                placed = True
                break
        if not placed:
            clusters.append({"ang": seg["ang"], "segs": [seg], "len": seg["len"]})
    clusters.sort(key=lambda c: -c["len"])
    if len(clusters) < 2 or _angle_diff(clusters[0]["ang"], clusters[1]["ang"]) < 30.0:
        return cx_box, cy_box, False, "single_direction"

    lines_fit = []
    for c in clusters[:2]:
        pts = []
        for s in c["segs"]:
            pts.append(s["p1"])
            pts.append(s["p2"])
        point, direction = _fit_line(pts)
        lines_fit.append((point, direction))

    cross = _intersect(lines_fit[0][0], lines_fit[0][1],
                       lines_fit[1][0], lines_fit[1][1])
    if cross is None:
        return cx_box, cy_box, False, "parallel"

    gx, gy = cross[0] + rx1, cross[1] + ry1
    # 交点不应离 bbox 中心太远
    if abs(gx - cx_box) > 0.75 * w + pad or abs(gy - cy_box) > 0.75 * h + pad:
        return cx_box, cy_box, False, "out_of_box"
    return float(gx), float(gy), True, "line_intersection"
