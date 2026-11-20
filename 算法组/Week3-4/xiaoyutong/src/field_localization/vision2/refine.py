"""几何细化：给定 YOLO 检出的一个框/点，在框内算精确交点，并判出 L / T / X。

为什么需要：
    YOLO 只给"这里有个标志"，框中心不等于真实交点；单类模型也不区分 L/T/X。
    细则明确要求："检测框中心不一定等于精确几何交点，必须做几何细化"。

做法：把框扩一圈做 ROI → 在 ROI 上用 markers_thick 的粗线拟合找交点并判型 →
      把坐标换算回原图，取离框中心最近的那个结果。
"""

import math

import numpy as np

try:
    from . import markers_thick as mt
except ImportError:  # pragma: no cover
    import markers_thick as mt


def refine(frame, u, v, box=None, pad_ratio=0.6, min_size=180, max_size=420):
    """返回 (type, x, y, conf) 或 None。type ∈ {L,T,X}。"""
    h, w = frame.shape[:2]
    if box is not None:
        x1, y1, x2, y2 = [float(t) for t in box]
        bw, bh = x2 - x1, y2 - y1
    else:
        bw = bh = min_size
        x1, y1, x2, y2 = u - bw / 2, v - bh / 2, u + bw / 2, v + bh / 2

    side = max(min_size, min(max_size, int(max(bw, bh) * (1 + 2 * pad_ratio))))
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    rx1 = int(max(0, round(cx - side / 2)))
    ry1 = int(max(0, round(cy - side / 2)))
    rx2 = int(min(w - 1, round(cx + side / 2)))
    ry2 = int(min(h - 1, round(cy + side / 2)))
    if rx2 - rx1 < 60 or ry2 - ry1 < 60:
        return None

    roi = frame[ry1:ry2, rx1:rx2]
    dets = mt.detect(roi)
    if not dets:
        return None

    best = None
    best_d = 1e9
    for d in dets:
        gx, gy = d["u"] + rx1, d["v"] + ry1
        dd = math.hypot(gx - u, gy - v)
        if dd < best_d:
            best_d, best = dd, (d, gx, gy)
    if best is None or best_d > 0.6 * side:
        return None

    d, gx, gy = best
    return d["type"], float(gx), float(gy), float(d.get("conf", 0.7))


def refine_all(frame, dets):
    """对一批检测结果做几何细化；判不出类型的丢掉。"""
    out = []
    for d in dets:
        r = refine(frame, d["u"], d["v"], box=d.get("bbox"))
        if r is None:
            continue
        typ, x, y, cf = r
        out.append({"type": d.get("type") or typ, "u": x, "v": y,
                    "conf": min(1.0, 0.5 * d.get("conf", 0.6) + 0.5 * cf),
                    "bbox": d.get("bbox")})
    return out
