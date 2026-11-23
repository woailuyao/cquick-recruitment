# -*- coding: utf-8 -*-
"""
场地标志几何细化：YOLO 检测框 -> 真实几何特征点像素坐标

细则 2.3 明确要求：不能把 bbox 中心当成 L/T/X 的精确交点。
本模块对每个检测框内的图像做局部几何处理：

    PenaltyPoint : 白色圆形/椭圆目标的质心
    L / T / X    : 白色线条 -> Hough 线段 -> 按方向聚类 -> 取两条主直线 -> 求交点
                   （L 取两腿交会顶点，T 取横竖交点，X 取两线交叉中心 —— 都是交点）

失败时回退到 bbox 中心，并把 refined 置 False，供上层降权或丢弃。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class RefineResult:
    u: float
    v: float
    ok: bool
    method: str
    n_lines: int = 0


def _white_mask(roi: np.ndarray) -> np.ndarray:
    """提取场地白线。兼顾 HSV（低饱和高亮）与灰度备用阈值。"""
    if roi.ndim == 2:
        gray = roi
        hsv = None
    else:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    mask = np.zeros(gray.shape, np.uint8)
    if hsv is not None:
        m_hsv = cv2.inRange(hsv, (0, 0, 105), (180, 85, 255))
        mask = cv2.bitwise_or(mask, m_hsv)
    m_gray = cv2.inRange(gray, 165, 255)
    mask = cv2.bitwise_or(mask, m_gray)

    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    return mask


def _fit_line(pts: np.ndarray) -> tuple[float, float, float]:
    """总体最小二乘拟合直线，返回 (a, b, c) 使 a*x + b*y + c = 0，且 a²+b²=1。"""
    pts = pts.astype(np.float64)
    mean = pts.mean(0)
    u, s, vt = np.linalg.svd(pts - mean)
    direction = vt[0]
    normal = np.array([-direction[1], direction[0]])
    n = np.linalg.norm(normal)
    if n < 1e-12:
        return 0.0, 1.0, -mean[1]
    normal /= n
    return float(normal[0]), float(normal[1]), float(-normal @ mean)


def _intersect(l1, l2) -> Optional[tuple[float, float]]:
    a1, b1, c1 = l1
    a2, b2, c2 = l2
    d = a1 * b2 - a2 * b1
    if abs(d) < 1e-9:
        return None
    x = (b1 * c2 - b2 * c1) / d
    y = (a2 * c1 - a1 * c2) / d
    return float(x), float(y)


def _segments_to_lines(mask: np.ndarray, min_len_ratio: float = 0.30):
    """Hough 线段 -> 按方向聚类 -> 每个簇拟合一条直线。"""
    h, w = mask.shape[:2]
    diag = math.hypot(h, w)
    min_len = max(8, int(diag * min_len_ratio))
    edges = cv2.Canny(mask, 50, 150)
    segs = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=max(15, min_len // 3),
                           minLineLength=min_len, maxLineGap=max(4, min_len // 3))
    if segs is None:
        return []
    # OpenCV 4 返回 (N,1,4)，OpenCV 5 返回 (N,4)，统一成 (N,4)
    segs = np.asarray(segs).reshape(-1, 4)

    # 每个线段投票到 0~180 度方向（模 180）
    buckets = [[] for _ in range(18)]
    for row in segs:
        x1, y1, x2, y2 = (float(row[0]), float(row[1]), float(row[2]), float(row[3]))
        ang = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0
        idx = int(ang // 10) % 18
        buckets[idx].append((x1, y1, x2, y2))

    # 合并相邻桶，避免方向落在桶边界被拆散
    clusters = []
    used = [False] * 18
    for i in range(18):
        if used[i] or not buckets[i]:
            continue
        group = list(buckets[i])
        used[i] = True
        for j in ((i + 1) % 18, (i - 1) % 18):
            if not used[j] and buckets[j]:
                group.extend(buckets[j])
                used[j] = True
        clusters.append((len(group), group))

    clusters.sort(key=lambda t: -t[0])
    lines = []
    for _, group in clusters[:3]:
        pts = []
        for x1, y1, x2, y2 in group:
            pts.append((x1, y1))
            pts.append((x2, y2))
            # 线段太长时在中间补点，提升拟合稳定性
            for t in (0.25, 0.5, 0.75):
                pts.append((x1 + (x2 - x1) * t, y1 + (y2 - y1) * t))
        if len(pts) >= 4:
            lines.append(_fit_line(np.array(pts, dtype=np.float64)))
    return lines


def _angle_between(l1, l2) -> float:
    """两条直线的夹角（0~90 度）。"""
    n1 = np.array(l1[:2])
    n2 = np.array(l2[:2])
    cos = abs(float(n1 @ n2))
    cos = min(1.0, max(0.0, cos))
    return math.degrees(math.acos(cos))


def refine_landmark(image: np.ndarray, bbox, cls_name: str,
                    pad_ratio: float = 0.15) -> RefineResult:
    """
    image: 完整 BGR 图像
    bbox: (x1, y1, x2, y2) 像素
    cls_name: LCross / TCross / XCross / PenaltyPoint
    返回像素坐标 (u, v) 与是否成功。
    """
    H, W = image.shape[:2]
    x1, y1, x2, y2 = [float(v) for v in bbox]
    bw, bh = max(1.0, x2 - x1), max(1.0, y2 - y1)
    pad_x, pad_y = bw * pad_ratio, bh * pad_ratio
    ix1 = int(max(0, math.floor(x1 - pad_x)))
    iy1 = int(max(0, math.floor(y1 - pad_y)))
    ix2 = int(min(W - 1, math.ceil(x2 + pad_x)))
    iy2 = int(min(H - 1, math.ceil(y2 + pad_y)))
    if ix2 - ix1 < 6 or iy2 - iy1 < 6:
        return RefineResult((x1 + x2) / 2, (y1 + y2) / 2, False, "bbox_too_small")

    roi = image[iy1:iy2, ix1:ix2]
    mask = _white_mask(roi)
    if int(mask.sum()) < 40:
        return RefineResult((x1 + x2) / 2, (y1 + y2) / 2, False, "no_white")

    if cls_name in ("PenaltyPoint", "PenalityPoint", "P"):
        # 点球点：白色块质心
        ys, xs = np.nonzero(mask)
        if xs.size < 6:
            return RefineResult((x1 + x2) / 2, (y1 + y2) / 2, False, "no_blob")
        cu, cv = float(xs.mean()), float(ys.mean())
        return RefineResult(ix1 + cu, iy1 + cv, True, "blob_centroid")

    lines = _segments_to_lines(mask)
    if len(lines) < 2:
        return RefineResult((x1 + x2) / 2, (y1 + y2) / 2, False,
                            "lines_insufficient", n_lines=len(lines))

    # 选夹角最大的一对直线（最稳定的交点）
    best = None
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            ang = _angle_between(lines[i], lines[j])
            if ang < 25.0:            # 近平行，交点不稳定
                continue
            pt = _intersect(lines[i], lines[j])
            if pt is None:
                continue
            # 交点必须落在 ROI 附近（允许一定外扩），否则视为误拟合
            margin = 0.6
            if not (-margin * bw <= pt[0] <= (1 + margin) * bw and
                    -margin * bh <= pt[1] <= (1 + margin) * bh):
                continue
            if best is None or ang > best[0]:
                best = (ang, pt)

    if best is None:
        return RefineResult((x1 + x2) / 2, (y1 + y2) / 2, False,
                            "no_valid_intersection", n_lines=len(lines))

    _, (cu, cv) = best
    return RefineResult(ix1 + cu, iy1 + cv, True, "line_intersection",
                        n_lines=len(lines))
