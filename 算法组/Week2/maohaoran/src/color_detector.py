"""HSV 颜色目标检测模块（任务 1/2 与 ROS2 发布节点共用）。

流程：
    BGR -> HSV -> 二值 mask -> 形态学 -> 外轮廓 -> 面积筛选 -> 中心点

约定：
    - 未检测到目标时 detected=False，u/v/area 全部为 0.0
      （与 TargetPosition.msg 的约定一致，避免把"没检测到"误当成左上角 (0,0)）。
    - 多个候选轮廓时选择面积最大的一个。
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

# 默认参数：蓝色目标的 HSV 范围（OpenCV 的 H 取值 0~179）
DEFAULT_HSV_LOWER = (100, 80, 40)
DEFAULT_HSV_UPPER = (130, 255, 255)
DEFAULT_MIN_AREA = 2000.0


@dataclass
class DetectionResult:
    """一帧图像（或一张图片）的检测结果。"""

    detected: bool
    u: float = 0.0          # 目标中心横坐标（像素，向右增大）
    v: float = 0.0          # 目标中心纵坐标（像素，向下增大）
    area: float = 0.0       # 目标轮廓面积（像素平方）
    contour: np.ndarray | None = None   # 选中的轮廓，用于可视化
    bbox: tuple | None = None           # (x, y, w, h) 外接矩形
    center_method: str = ""             # "moments" 或 "bbox"，记录中心点算法
    candidates: list = field(default_factory=list)  # 所有通过面积筛选的轮廓


def build_mask(bgr, lower=DEFAULT_HSV_LOWER, upper=DEFAULT_HSV_UPPER):
    """把 BGR 图像转成 HSV 后，按上下界生成二值掩膜。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower_np = np.array(lower, dtype=np.uint8)
    upper_np = np.array(upper, dtype=np.uint8)
    return cv2.inRange(hsv, lower_np, upper_np)


def apply_morphology(mask, open_size=5, close_size=5):
    """开运算去小噪点，闭运算补目标内部的小洞。size<=1 表示不执行该操作。"""
    result = mask
    if open_size and open_size > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_size, open_size))
        result = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel)
    if close_size and close_size > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size))
        result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel)
    return result


def contour_center(contour):
    """用轮廓矩求中心点；m00 为 0 时退化为外接矩形中心。"""
    moments = cv2.moments(contour)
    if moments["m00"] > 0:
        u = moments["m10"] / moments["m00"]
        v = moments["m01"] / moments["m00"]
        return float(u), float(v), "moments"
    x, y, w, h = cv2.boundingRect(contour)
    return x + w / 2.0, y + h / 2.0, "bbox"


def detect(bgr, lower=DEFAULT_HSV_LOWER, upper=DEFAULT_HSV_UPPER,
           min_area=DEFAULT_MIN_AREA, use_morphology=True,
           open_size=5, close_size=5):
    """检测指定颜色的目标，返回 (DetectionResult, mask)。

    mask 也返回给调用方，方便任务 2 的可视化和参数分析。
    """
    mask = build_mask(bgr, lower, upper)
    if use_morphology:
        mask = apply_morphology(mask, open_size, close_size)

    # RETR_EXTERNAL：只取最外层轮廓，目标内部的高光洞不会把轮廓切断
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 面积筛选：去掉明显是噪点的小轮廓
    candidates = [c for c in contours if cv2.contourArea(c) >= min_area]

    if not candidates:
        return DetectionResult(detected=False), mask

    # 多个候选时选面积最大的
    target = max(candidates, key=cv2.contourArea)
    u, v, method = contour_center(target)
    area = float(cv2.contourArea(target))
    bbox = cv2.boundingRect(target)

    result = DetectionResult(
        detected=True, u=u, v=v, area=area,
        contour=target, bbox=bbox,
        center_method=method, candidates=candidates,
    )
    return result, mask
