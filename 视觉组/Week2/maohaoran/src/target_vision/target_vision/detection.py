"""HSV 颜色目标检测模块（ROS2 节点使用）。

与 Windows 侧 src/color_detector.py 逻辑一致，复制一份到包内，
保证这个 ROS2 包可以独立编译、不依赖外部目录。

流程：BGR -> HSV -> mask -> 形态学 -> 外轮廓 -> 面积筛选 -> 中心点
未检测到时 u/v/area 全为 0.0，由 detected 字段判断本帧是否有效。
"""

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class DetectionResult:
    detected: bool
    u: float = 0.0
    v: float = 0.0
    area: float = 0.0
    contour: np.ndarray | None = None
    bbox: tuple | None = None
    center_method: str = ""
    candidates: list = field(default_factory=list)


def build_mask(bgr, lower, upper):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv,
                       np.array(lower, dtype=np.uint8),
                       np.array(upper, dtype=np.uint8))


def apply_morphology(mask, open_size=5, close_size=5):
    result = mask
    if open_size and open_size > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (open_size, open_size))
        result = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel)
    if close_size and close_size > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (close_size, close_size))
        result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel)
    return result


def contour_center(contour):
    moments = cv2.moments(contour)
    if moments["m00"] > 0:
        return (float(moments["m10"] / moments["m00"]),
                float(moments["m01"] / moments["m00"]),
                "moments")
    x, y, w, h = cv2.boundingRect(contour)
    return x + w / 2.0, y + h / 2.0, "bbox"


def detect(bgr, lower, upper, min_area,
           use_morphology=True, open_size=5, close_size=5):
    mask = build_mask(bgr, lower, upper)
    if use_morphology:
        mask = apply_morphology(mask, open_size, close_size)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    candidates = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not candidates:
        return DetectionResult(detected=False), mask

    target = max(candidates, key=cv2.contourArea)
    u, v, method = contour_center(target)
    return DetectionResult(
        detected=True,
        u=u, v=v,
        area=float(cv2.contourArea(target)),
        contour=target,
        bbox=cv2.boundingRect(target),
        center_method=method,
        candidates=candidates,
    ), mask
