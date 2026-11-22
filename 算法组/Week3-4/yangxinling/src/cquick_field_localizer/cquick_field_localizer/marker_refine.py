#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
marker_refine.py —— 把检测框细化成"真正的交点像素"（任务2 的硬性要求）

考核文档明确写了：
    "不能只把 bbox 中心当标志点，需要在 bbox 内做几何细化，输出真实交点像素"

为什么必须细化：
    L/T/X 的 bbox 中心通常偏离真实交点 5~20 像素（因为框里有半截线、
    草皮纹理、以及近大远小的透视）。在 5 m 处，10 像素 ≈ 3 cm，其实还好；
    但在 1~2 m 的近处，10 像素能到 10 cm 以上，而且不同标志的偏差方向不同，
    会系统性地把位姿拉偏。

本模块提供两类细化：
    L / T / X 型交点：白线掩膜 → Canny → 概率霍夫 → 按方向聚类 → 两条主直线求交
    点球点(PenaltyPoint)：红色圆点 → HSV 红色掩膜 → 质心

返回 dict：uv(细化后像素)、ok、quality(0~1)、method、fallback_reason
"""

import math

import cv2
import numpy as np

WHITE_LINE_CLASSES = (0, 1, 2)      # LCross, TCross, XCross
PENALTY_POINT_CLASS = 3


# ---------------------------------------------------------------------------
def _white_mask(bgr):
    """
    草皮上的白线：用"白顶帽"提取细而亮的线状结构 + 低饱和约束。

    为什么不用固定阈值：实拍视频里光照差异很大（有的帧整体过曝、草皮发白），
    固定 V 阈值会把草皮也当成白线。白顶帽只保留"比局部更亮的细结构"，
    对整体明暗不敏感，是这类场地线检测最稳的经典做法。
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    _h, s, v = cv2.split(hsv)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, k)

    thr_tophat = max(18.0, float(np.percentile(tophat, 92)))
    mask = ((tophat > thr_tophat) & (s < 120) & (v > 90)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return mask


def _red_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (0, 70, 50), (12, 255, 255))
    m2 = cv2.inRange(hsv, (168, 70, 50), (180, 255, 255))
    mask = cv2.bitwise_or(m1, m2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return mask


def _line_hypotheses(mask, gray):
    """
    用"梯度方向直方图"找出 ROI 里的两条主直线，并返回 (方向, 偏移) 形式。

    为什么不用 cv2.HoughLinesP：
        实拍场地线的白漆在草皮上常常断续（被磨损、被遮挡、过曝），
        Hough 需要足够长的连续线段，经常只检出一条。
        而梯度方向直方图用的是**每一个白线像素**，断线也能统计出方向；
        方向确定后，把像素投影到法向做一维直方图就能拿到直线的位置。

    返回 [(nx, ny, c), ...]，直线方程 nx*x + ny*y = c（像素坐标）。
    """
    ys, xs = np.nonzero(mask)
    if len(xs) < 40:
        return []
    mag = np.zeros_like(gray, dtype=np.float32)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    mag = cv2.magnitude(gx, gy)

    m = mag[ys, xs]
    if float(m.sum()) <= 1e-6:
        return []
    ang = np.arctan2(gy[ys, xs], gx[ys, xs])          # 梯度方向 = 线法向
    ang = np.mod(ang, np.pi)                          # 线方向无向，折到 [0,180)

    nb = 180
    hist = np.zeros(nb, dtype=np.float64)
    idx = np.clip((ang / np.pi * nb).astype(int), 0, nb - 1)
    np.add.at(hist, idx, m)

    # 平滑后找峰
    kern = np.array([1.0, 2.0, 3.0, 2.0, 1.0])
    kern /= kern.sum()
    hs = np.convolve(np.r_[hist[-2:], hist, hist[:2]], kern, mode="same")[2:-2]

    peaks = []
    order = np.argsort(hs)[::-1]
    for b in order:
        if hs[b] <= 0:
            break
        theta = (b + 0.5) / nb * np.pi
        if any(min(abs(theta - t), np.pi - abs(theta - t)) < np.deg2rad(20) for t, _ in peaks):
            continue
        peaks.append((theta, float(hs[b])))
        if len(peaks) >= 3:
            break

    lines = []
    for theta, _w in peaks:
        nx, ny = math.cos(theta), math.sin(theta)
        p = xs * nx + ys * ny
        # 一维直方图找线位置（带宽 2 px）
        lo, hi = float(p.min()), float(p.max())
        if hi - lo < 3:
            continue
        bins = np.arange(lo, hi + 2.0, 2.0)
        h1, edges = np.histogram(p, bins=bins)
        b = int(np.argmax(h1))
        c0 = 0.5 * (edges[b] + edges[b + 1])
        inl = np.abs(p - c0) < 7.0
        if inl.sum() < 25:
            continue
        c = float(np.median(p[inl]))                  # 用中位数抗离群点
        inl = np.abs(p - c) < 6.0
        if inl.sum() < 20:
            continue
        c = float(np.mean(p[inl]))
        lines.append((nx, ny, c, int(inl.sum())))
    return lines


def _intersect_nc(l1, l2):
    """两条 (nx, ny, c) 形式的直线求交。"""
    n1 = np.array([l1[0], l1[1]])
    n2 = np.array([l2[0], l2[1]])
    A = np.vstack([n1, n2])
    det = np.linalg.det(A)
    if abs(det) < 1e-3:                               # 两线几乎平行
        return None
    return np.linalg.solve(A, np.array([l1[2], l2[2]]))


# ---------------------------------------------------------------------------
def refine_marker(frame_bgr, bbox, cls_id, margin=0.3):
    """
    frame_bgr : 原始 BGR 图（未去畸变；这里在"原始像素坐标系"里细化，
                之后交给 CameraModel 一起做去畸变，避免二次插值误差累积）
    bbox      : (x1, y1, x2, y2) 像素
    cls_id    : 0=L 1=T 2=X 3=PenaltyPoint

    返回 dict:
        uv, ok, quality, method, fallback_reason, roi_bbox
    """
    H, W = frame_bgr.shape[:2]
    x1, y1, x2, y2 = [float(v) for v in bbox]
    x1, x2 = max(0.0, min(x1, x2)), min(float(W - 1), max(x1, x2))
    y1, y2 = max(0.0, min(y1, y2)), min(float(H - 1), max(y1, y2))
    bw, bh = x2 - x1, y2 - y1
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

    # 扩一点边，保证交点两侧都有线可用（bbox 常常只框住交点一角）
    ex = max(6.0, bw * margin)
    ey = max(6.0, bh * margin)
    rx1, ry1 = int(max(0, x1 - ex)), int(max(0, y1 - ey))
    rx2, ry2 = int(min(W, x2 + ex)), int(min(H, y2 + ey))
    roi = frame_bgr[ry1:ry2, rx1:rx2]
    if roi.size == 0:
        return {"uv": (cx, cy), "ok": False, "quality": 0.0,
                "method": "bbox_center", "fallback_reason": "empty_roi",
                "roi_bbox": (rx1, ry1, rx2, ry2)}

    # ---------------- 点球点：红色质心 ----------------
    if cls_id == PENALTY_POINT_CLASS:
        mask = _red_mask(roi)
        n = int(cv2.countNonZero(mask))
        if n >= 8:
            ys, xs = np.nonzero(mask)
            uv = (float(xs.mean() + rx1), float(ys.mean() + ry1))
            q = min(1.0, n / 60.0)
            return {"uv": uv, "ok": True, "quality": q, "method": "red_centroid",
                    "fallback_reason": "", "roi_bbox": (rx1, ry1, rx2, ry2)}

    # ---------------- L/T/X：两条主直线求交 ----------------
    if cls_id in WHITE_LINE_CLASSES:
        mask = _white_mask(roi)
        n = int(cv2.countNonZero(mask))
        if n >= 60:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            lines = _line_hypotheses(mask, gray)
            best = None
            for i in range(len(lines)):
                for k in range(i + 1, len(lines)):
                    a1 = math.atan2(lines[i][1], lines[i][0])
                    a2 = math.atan2(lines[k][1], lines[k][0])
                    sep = abs(math.degrees(a1 - a2)) % 180.0
                    sep = min(sep, 180.0 - sep)
                    if sep < 25.0:                    # 夹角太小 → 其实是同一条线的两条边
                        continue
                    p = _intersect_nc(lines[i], lines[k])
                    if p is None:
                        continue
                    u = float(p[0] + rx1)
                    v = float(p[1] + ry1)
                    # 交点必须落在扩边后的 ROI 内，否则是"远处延长线相交"，不可信
                    if not (rx1 - 2 <= u <= rx2 + 2 and ry1 - 2 <= v <= ry2 + 2):
                        continue
                    support = lines[i][3] + lines[k][3]      # 两条线的内点数之和
                    score = support * (0.5 + 0.5 * min(sep, 90.0) / 90.0)
                    if best is None or score > best[0]:
                        best = (score, u, v, support, sep)
            if best is not None:
                _score, u, v, support, sep = best
                q = min(1.0, support / 120.0) * min(1.0, sep / 60.0)
                return {"uv": (u, v), "ok": True, "quality": float(q),
                        "method": "line_intersection", "fallback_reason": "",
                        "roi_bbox": (rx1, ry1, rx2, ry2)}

    # ---------------- 兜底 ----------------
    return {"uv": (cx, cy), "ok": False, "quality": 0.0, "method": "bbox_center",
            "fallback_reason": "mask_or_lines_insufficient",
            "roi_bbox": (rx1, ry1, rx2, ry2)}


# ---------------------------------------------------------------------------
def _cli():
    import argparse
    import glob
    ap = argparse.ArgumentParser(description="标志点几何细化（单点调试）")
    ap.add_argument("--image", required=True)
    ap.add_argument("--bbox", nargs=4, type=float, required=True, help="x1 y1 x2 y2")
    ap.add_argument("--cls", type=int, default=0, help="0=L 1=T 2=X 3=PenaltyPoint")
    ap.add_argument("--out", default="marker_refine_debug.jpg")
    a = ap.parse_args()
    img = cv2.imread(a.image)
    if img is None:
        raise SystemExit("读不到图片 %s" % a.image)
    r = refine_marker(img, a.bbox, a.cls)
    print("method=%s ok=%s quality=%.3f uv=(%.2f, %.2f)" %
          (r["method"], r["ok"], r["quality"], r["uv"][0], r["uv"][1]))
    vis = img.copy()
    x1, y1, x2, y2 = [int(v) for v in a.bbox]
    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
    cv2.circle(vis, (int(round(r["uv"][0])), int(round(r["uv"][1]))), 5, (0, 0, 255), -1)
    cv2.putText(vis, r["method"], (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.imwrite(a.out, vis)
    print("调试图: %s" % a.out)


if __name__ == "__main__":
    _cli()
