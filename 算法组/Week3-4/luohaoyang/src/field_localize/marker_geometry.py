"""把 YOLO 的 bbox 细化为真正的几何特征点（线段交点）。

背景：L/T/X 的检测框动辄占半屏，bbox 中心不等于白线交点。
本模块在框内做白线分割 + 直线拟合 + 交点计算，输出 landmark_pixel=(u,v)。

对外只有一个函数：
    refine_marker_point(frame, box, marker_type="LCross") -> dict
返回:
    {"u": float, "v": float, "ok": bool, "reason": str, "n_lines": int,
     "angle_sep": float, "method": "intersection"|"centroid"|"bbox_center"}
"""

from __future__ import annotations

import math

import cv2
import numpy as np


# ----------------------------------------------------------------------
def _white_mask(bgr, sat_max=90, val_min=135):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    mask = ((s <= sat_max) & (v >= val_min)).astype(np.uint8) * 255
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def _segments(mask):
    h, w = mask.shape[:2]
    short = min(h, w)
    out = cv2.HoughLinesP(
        mask, 1, np.pi / 180.0,
        threshold=max(18, int(short * 0.12)),
        minLineLength=max(20, int(short * 0.18)),
        maxLineGap=max(6, int(short * 0.05)),
    )
    segs = []
    if out is not None:
        for l in out.reshape(-1, 4):
            x1, y1, x2, y2 = map(float, l)
            length = math.hypot(x2 - x1, y2 - y1)
            if length >= 15:
                segs.append((x1, y1, x2, y2, length))
    return segs


def _cluster_by_angle(segs, tol_deg=12.0):
    clusters = []  # [{"theta": rad, "segs": [...]}]
    tol = math.radians(tol_deg)
    for seg in sorted(segs, key=lambda s: -s[4]):
        theta = math.atan2(seg[3] - seg[1], seg[2] - seg[0]) % math.pi
        placed = False
        for c in clusters:
            d = abs(theta - c["theta"])
            d = min(d, math.pi - d)
            if d <= tol:
                w0 = c["weight"]
                c["theta"] = (c["theta"] * w0 + theta * seg[4]) / (w0 + seg[4])
                c["weight"] += seg[4]
                c["segs"].append(seg)
                placed = True
                break
        if not placed:
            clusters.append({"theta": theta, "weight": seg[4], "segs": [seg]})
    clusters.sort(key=lambda c: -c["weight"])
    return clusters


def _fit_line(segs):
    pts = []
    for x1, y1, x2, y2, L in segs:
        pts.append([x1, y1])
        pts.append([x2, y2])
    pts = np.array(pts, dtype=np.float32)
    if len(pts) < 2:
        return None
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    return (float(x0), float(y0), float(vx), float(vy))


def _intersect(l1, l2):
    x1, y1, vx1, vy1 = l1
    x2, y2, vx2, vy2 = l2
    det = vx1 * (-vy2) - vy1 * (-vx2)
    if abs(det) < 1e-9:
        return None
    rx, ry = x2 - x1, y2 - y1
    t = (rx * (-vy2) - ry * (-vx2)) / det
    return (x1 + t * vx1, y1 + t * vy1)


def _angle_sep_deg(l1, l2):
    a1 = math.atan2(l1[3], l1[2])
    a2 = math.atan2(l2[3], l2[2])
    d = abs(a1 - a2) % math.pi
    return math.degrees(min(d, math.pi - d))


# ----------------------------------------------------------------------
def refine_marker_point(frame, box, marker_type="LCross", pad=0.10,
                        min_angle_sep=25.0, allow_bbox_fallback=True):
    """box = (x1, y1, x2, y2) 像素坐标。"""
    H, W = frame.shape[:2]
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    bw, bh = max(1, x2 - x1), max(1, y2 - y1)
    px, py = int(bw * pad), int(bh * pad)
    rx1, ry1 = max(0, x1 - px), max(0, y1 - py)
    rx2, ry2 = min(W, x2 + px), min(H, y2 + py)
    roi = frame[ry1:ry2, rx1:rx2]
    cu, cv_ = (x1 + x2) / 2.0, (y1 + y2) / 2.0

    fallback = {"u": cu, "v": cv_, "ok": False, "reason": "fallback",
                "n_lines": 0, "angle_sep": 0.0, "method": "bbox_center"}
    if roi.size == 0:
        return {**fallback, "reason": "empty_roi"}

    mask = _white_mask(roi)

    # 点球点：用白色斑块质心，不做线拟合
    if marker_type == "PenaltyPoint":
        n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
        best, best_d = None, 1e9
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < 25:
                continue
            cx_, cy_ = cents[i]
            d = math.hypot(cx_ - roi.shape[1] / 2, cy_ - roi.shape[0] / 2)
            if d < best_d:
                best_d, best = d, (cx_, cy_)
        if best is None:
            return {**fallback, "reason": "no_blob"} if allow_bbox_fallback else None
        return {"u": best[0] + rx1, "v": best[1] + ry1, "ok": True,
                "reason": "blob_centroid", "n_lines": 0, "angle_sep": 0.0,
                "method": "centroid"}

    segs = _segments(mask)
    if len(segs) < 2:
        return ({**fallback, "reason": "no_lines"} if allow_bbox_fallback
                else None)

    clusters = _cluster_by_angle(segs)
    if len(clusters) < 2:
        return ({**fallback, "reason": "single_direction"} if allow_bbox_fallback
                else None)

    l1 = _fit_line(clusters[0]["segs"])
    l2 = None
    sep = 0.0
    for c in clusters[1:]:
        cand = _fit_line(c["segs"])
        if cand is None:
            continue
        s = _angle_sep_deg(l1, cand)
        if s >= min_angle_sep:
            l2, sep = cand, s
            break
    if l2 is None:
        return ({**fallback, "reason": "no_second_direction"} if allow_bbox_fallback
                else None)

    p = _intersect(l1, l2)
    if p is None:
        return ({**fallback, "reason": "parallel"} if allow_bbox_fallback else None)

    u, v = p[0] + rx1, p[1] + ry1
    if not (0 <= u < W and 0 <= v < H):
        return ({**fallback, "reason": "outside_image"} if allow_bbox_fallback else None)

    # 交点离框中心太远 -> 大概率拟合错了
    tol = 0.9 * max(bw, bh) + 10
    if math.hypot(u - cu, v - cv_) > tol:
        return ({**fallback, "reason": "too_far"} if allow_bbox_fallback else None)

    return {"u": float(u), "v": float(v), "ok": True, "reason": "ok",
            "n_lines": len(segs), "angle_sep": float(sep), "method": "intersection"}


# ----------------------------------------------------------------------
def draw_debug(frame, box, refined, color=(0, 255, 0)):
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cu, cv_ = int((x1 + x2) / 2), int((y1 + y2) / 2)
    cv2.drawMarker(frame, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 10, 1)
    if refined and refined.get("ok"):
        p = (int(round(refined["u"])), int(round(refined["v"])))
        cv2.circle(frame, p, 4, (0, 255, 255), -1)
        cv2.line(frame, (cu, cv_), p, (0, 255, 255), 1)
    return frame