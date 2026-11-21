# -*- coding: utf-8 -*-
"""
line_utils.py —— 白线掩膜与"线的几何中心线"工具

white_mask()      画面 -> 白线二值图（草皮门 + HSV 白阈值）
_line_segments()  白线 -> 直线段（用距离变换脊线，不是 Canny 边缘）
_seg_intersect()  两条线段所在直线的交点

用"脊线"而非 Canny：场地线很粗，Canny 取到的是两侧边界，求交会偏到
线边 20~50 px；距离变换的局部极大值才是中心线，用它求交才落在真实交会处。
"""
import math

import cv2
import numpy as np

DEFAULT_PARAMS = dict(
    # ---- 白色阈值（低饱和 + 高亮度）----
    s_max=90, v_min=140,
    v_auto=True, v_pct=80,        # 亮度门限抬到 V 的 80 分位，应对曝光差异
    # ---- 草皮门：只有落在草地上的白像素才算场地线，用于剔除白墙等高亮背景 ----
    turf_gate=True, turf_h_lo=25, turf_h_hi=110, turf_s_min=15, turf_v_min=35,
    turf_close=71, turf_pad=16, turf_min_cover=0.12,
    polarity="bright",            # bright=亮线暗地；dark=暗线亮地
    open_k=3, close_k=5,          # 形态学开/闭核
    roi_top=0.05,                 # 切掉画面顶部 5%（墙/观众席）；0.18 会把远端 L/T 切掉导致细化失败
    # ---- 霍夫直线 ----
    hough_thresh=20,              # 累加阈值（中轴线像素少，比边缘法低）
    min_line_len=25,
    max_line_gap=20,              # 中轴线在交点处会断，gap 要给大
)


def white_mask(bgr, p=DEFAULT_PARAMS):
    """得到"只有白色场地线"的二值图：HSV 白阈值 -> 草皮门 -> 形态学清理。"""
    if p["polarity"] == "dark":
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 25, 7)
    else:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        v_min = int(p["v_min"])
        if p["v_auto"]:
            v_min = min(235, max(v_min, int(np.percentile(hsv[:, :, 2], p["v_pct"]))))
        mask = cv2.inRange(hsv, np.array([0, 0, v_min], np.uint8),
                           np.array([180, p["s_max"], 255], np.uint8))

        if p["turf_gate"]:
            hub = np.array([p["turf_h_lo"], p["turf_s_min"], p["turf_v_min"]], np.uint8)
            hlb = np.array([p["turf_h_hi"], 255, 255], np.uint8)
            turf = cv2.inRange(hsv, hub, hlb)
            if float((turf > 0).mean()) >= p["turf_min_cover"]:
                # 草皮门是粗区域门，在 1/4 分辨率上计算再放大以节省耗时
                S = 4
                h_, w_ = turf.shape[:2]
                small = cv2.resize(turf, (max(1, w_ // S), max(1, h_ // S)),
                                   interpolation=cv2.INTER_AREA)
                kc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                               (max(3, p["turf_close"] // S),) * 2)
                small = cv2.morphologyEx(small, cv2.MORPH_CLOSE, kc)
                kd = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                               (max(3, (2 * p["turf_pad"] + 1) // S),) * 2)
                small = cv2.dilate(small, kd)
                turf = cv2.resize(small, (w_, h_), interpolation=cv2.INTER_NEAREST)
                turf = np.where(turf > 0, np.uint8(255), np.uint8(0))
                mask = cv2.bitwise_and(mask, turf)

    ko = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p["open_k"],) * 2)
    kc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p["close_k"],) * 2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ko)     # 去噪点
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kc)    # 补断线
    mask[:int(mask.shape[0] * p["roi_top"]), :] = 0
    return mask


def _ridge_image(mask, D):
    """掩膜的中轴线：距离变换的局部极大值脊线。"""
    ridge = np.zeros_like(mask)
    c = D[1:-1, 1:-1]
    ridge[1:-1, 1:-1] = ((c >= D[:-2, 1:-1]) & (c >= D[2:, 1:-1]) &
                         (c >= D[1:-1, :-2]) & (c >= D[1:-1, 2:]) &
                         (mask[1:-1, 1:-1] > 0)).astype(np.uint8) * 255
    return ridge


def _line_segments(mask, p=DEFAULT_PARAMS, D=None):
    """从中轴线取直线段，返回 [(x1,y1,x2,y2), ...]。"""
    if D is None:
        D = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    ridge = _ridge_image(mask, D)
    segs = cv2.HoughLinesP(ridge, 1, np.pi / 180, threshold=p["hough_thresh"],
                           minLineLength=p["min_line_len"], maxLineGap=p["max_line_gap"])
    if segs is None:
        return []
    # OpenCV 4.x 返回 (N,1,4)，5.x 返回 (N,4)，统一 reshape(-1,4)
    return [(float(a), float(b), float(c), float(d))
            for a, b, c, d in np.asarray(segs).reshape(-1, 4)]


def _seg_intersect(s1, s2):
    """两条线段所在直线的交点；平行时返回 None。"""
    x1, y1, x2, y2 = s1
    x3, y3, x4, y4 = s2
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-9:
        return None
    a = x1 * y2 - y1 * x2
    b = x3 * y4 - y3 * x4
    return ((a * (x3 - x4) - (x1 - x2) * b) / den,
            (a * (y3 - y4) - (y1 - y2) * b) / den)
