"""纯 OpenCV 检测流程，可同时用于离线图片和 ROS 2 图像回调。"""
from dataclasses import dataclass
import math
from numbers import Integral, Real

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorConfig:
    hsv_lower: tuple = (95, 80, 70)
    hsv_upper: tuple = (130, 255, 255)
    min_area: float = 500.0
    open_kernel: int = 3
    close_kernel: int = 5

    def validate(self) -> None:
        """常规 8 位 HSV 的 H 上限为 179，S 和 V 上限为 255。"""
        for name, values in (("hsv_lower", self.hsv_lower), ("hsv_upper", self.hsv_upper)):
            if not isinstance(values, (tuple, list)) or len(values) != 3:
                raise ValueError(f"{name} must contain three integers")
            for value, maximum in zip(values, (179, 255, 255)):
                if isinstance(value, bool) or not isinstance(value, Integral) or not 0 <= value <= maximum:
                    raise ValueError(f"{name}: H must be 0..179; S and V must be 0..255")
        if any(low > high for low, high in zip(self.hsv_lower, self.hsv_upper)):
            raise ValueError("Each HSV lower bound must be <= its upper bound")
        if isinstance(self.min_area, bool) or not isinstance(self.min_area, Real) or not math.isfinite(self.min_area) or self.min_area <= 0:
            raise ValueError("min_area must be a finite positive number")
        for name, value in (("open_kernel", self.open_kernel), ("close_kernel", self.close_kernel)):
            if isinstance(value, bool) or not isinstance(value, Integral) or value < 1 or value % 2 == 0:
                raise ValueError(f"{name} must be a positive odd integer; 1 disables this operation")


@dataclass
class Detection:
    detected: bool
    u: float
    v: float
    area: float
    candidates: list[dict]
    raw_mask: np.ndarray
    mask: np.ndarray
    annotated: np.ndarray


def detect(bgr: np.ndarray, config: DetectorConfig) -> Detection:
    """输出最大合格色块；它只是颜色候选，不保证是指定物体。原图不修改。"""
    config.validate()
    if not isinstance(bgr, np.ndarray) or bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3 or min(bgr.shape[:2]) == 0:
        raise ValueError("Input must be a nonempty uint8 BGR image with three channels")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    raw_mask = cv2.inRange(hsv, np.array(config.hsv_lower, np.uint8), np.array(config.hsv_upper, np.uint8))
    mask = raw_mask.copy()
    # 开运算去小白噪声，闭运算填小黑孔；边长 1 时明确跳过该操作。
    for size, operation in ((config.open_kernel, cv2.MORPH_OPEN), (config.close_kernel, cv2.MORPH_CLOSE)):
        if size > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
            mask = cv2.morphologyEx(mask, operation, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    qualified = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < config.min_area:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        # 质心坐标是 (列 u, 行 v)，不能把数组的行列顺序直接当作坐标顺序。
        candidate = {"u": float(moments["m10"] / moments["m00"]), "v": float(moments["m01"] / moments["m00"]), "area": area,
                     "bbox": {"x": x, "y": y, "width": width, "height": height}}
        qualified.append((candidate, contour))
    # 同面积时按框位置、大小和质心排序，不依赖 findContours 的返回顺序。
    qualified.sort(key=lambda item: (-item[0]["area"], item[0]["bbox"]["x"], item[0]["bbox"]["y"], item[0]["bbox"]["width"], item[0]["bbox"]["height"], item[0]["u"], item[0]["v"]))
    annotated = bgr.copy()
    candidates = []
    for index, (candidate, contour) in enumerate(qualified):
        candidate.update({"rank": index + 1, "selected": index == 0})
        candidates.append(candidate)
        box = candidate["bbox"]
        x, y, width, height = box["x"], box["y"], box["width"], box["height"]
        color = (0, 220, 0) if index == 0 else (210, 210, 150)
        cv2.rectangle(annotated, (x, y), (x + width - 1, y + height - 1), color, 2 if index == 0 else 1)
        cv2.putText(annotated, f"#{index + 1}", (x, max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        if index == 0:
            cv2.drawContours(annotated, [contour], -1, color, 2)
            cv2.drawMarker(annotated, (round(candidate["u"]), round(candidate["v"])), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)

    if candidates:
        selected = candidates[0]
        u, v, area = selected["u"], selected["v"], selected["area"]
        label = f"SELECTED u={u:.1f} v={v:.1f} area={area:.1f}"
    else:
        u = v = area = 0.0
        label = "NO TARGET"
    # 信息写在下边缘，避免与样例图顶部的来源说明重叠。
    label_y = max(15, bgr.shape[0] - 12)
    label_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0][0]
    cv2.rectangle(annotated, (4, label_y - 19), (min(bgr.shape[1] - 1, label_width + 16), min(bgr.shape[0] - 1, label_y + 6)), (30, 30, 30), -1)
    cv2.putText(annotated, label, (10, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return Detection(bool(candidates), u, v, area, candidates, raw_mask, mask, annotated)
