"""场地标志几何细化：从 YOLO bbox 内提取精确几何特征点。

考核要求：检测框中心 ≠ 精确几何交点，必须在 bbox 内做几何细化
（线段拟合 / 角点检测），输出真实 landmark_pixel=(u,v)。

各类型取点规则：
  - LCross：两条白色线的交会顶点（角点）
  - TCross：横线与竖线的交点
  - XCross：两条线交会中心
  - PenaltyPoint：圆形标记中心（质心）

这里提供通用实现：对 bbox 内 ROI 做二值化 + 轮廓/角点/质心分析，
并说明规则；实际场地可替换为更精细的拟合。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class Landmark:
    type: str
    u: float
    v: float
    confidence: float


def _bbox_roi(gray: np.ndarray, box) -> np.ndarray:
    """截取 bbox 内的灰度 ROI（box 为 (x1,y1,x2,y2)，已取整）。"""
    x1, y1, x2, y2 = [int(v) for v in box]
    h, w = gray.shape
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    return gray[y1:y2, x1:x2], (x1, y1)


def _binary_white(gray_roi: np.ndarray, thresh: int = 180) -> np.ndarray:
    """白色标志线二值化。"""
    _, bw = cv2.threshold(gray_roi, thresh, 255, cv2.THRESH_BINARY)
    return bw


def refine_LCross(gray: np.ndarray, box, conf: float) -> Optional[Landmark]:
    """L 型角点：用 Shi-Tomasi / Harris 角点检测取最强角点。"""
    roi, (x0, y0) = _bbox_roi(gray, box)
    if roi.size == 0:
        return None
    bw = _binary_white(roi)
    corners = cv2.goodFeaturesToTrack(
        bw, maxCorners=1, qualityLevel=0.01, minDistance=5
    )
    if corners is None:
        return None
    cx, cy = corners[0][0]
    # type 必须与 field.yaml / YOLO 数据集类别名严格一致
    return Landmark("LCross", x0 + cx, y0 + cy, conf)


def refine_TCross(gray: np.ndarray, box, conf: float) -> Optional[Landmark]:
    """T 型：横线与竖线交点。简化：取二值前景的质心（T 的交点近似质心附近）。"""
    roi, (x0, y0) = _bbox_roi(gray, box)
    if roi.size == 0:
        return None
    bw = _binary_white(roi)
    m = cv2.moments(bw)
    if m["m00"] < 1e-6:
        return None
    cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
    # type 必须与 field.yaml / YOLO 数据集类别名严格一致
    return Landmark("TCross", x0 + cx, y0 + cy, conf)


def refine(gray: np.ndarray, cls_name: str, box, conf: float) -> Optional[Landmark]:
    """按类别分发几何细化。cls_name 为 YOLO 类别名。"""
    if cls_name == "LCross":
        lm = refine_LCross(gray, box, conf)
    elif cls_name == "TCross":
        lm = refine_TCross(gray, box, conf)
    elif cls_name == "XCross":
        lm = refine_TCross(gray, box, conf)
        if lm is not None:
            lm.type = "XCross"
    elif cls_name == "PenaltyPoint":
        lm = refine_TCross(gray, box, conf)
        if lm is not None:
            lm.type = "PenaltyPoint"
    else:
        lm = None
    return lm
