"""颜色目标检测：复用第二周任务1 的核心逻辑，参数化以便 ROS2 运行时调。

坐标约定（与任务1/2 一致）：
    - 左上角为原点；u 向右为正，v 向下为正；
    - u/v 是「像素坐标」，不是机器人真实世界坐标（转换需标定，本周不要求）。
"""
import os

import numpy as np


def detect_target(image_path, hsv_lower, hsv_upper, min_area, kernel_size=5):
    """检测图中面积最大的颜色目标，返回 dict。

    返回值：
        detected : bool  是否检测到
        u, v     : float 中心点像素坐标（未检测到时为 0）
        area     : float 最大轮廓面积（未检测到时为 0）
        error    : str   仅在读图失败时存在，便于节点报错

    说明：
        - HSV 上下界、最小面积都来自 ROS2 参数，可运行时调整（加分项）；
        - 中心点用轮廓矩(moments)，m00==0 时退回外接矩形中心；
        - 未检测到时 detected=False 且 u/v/area 全为 0，订阅方靠 detected 判断。
    """
    # 仅在真正需要检测时才导入 cv2（演示模式无需安装 OpenCV）
    import cv2

    img = cv2.imread(image_path)
    if img is None:
        return {"detected": False, "u": 0.0, "v": 0.0, "area": 0.0,
                "error": "cv2.imread failed: " + image_path}

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array(hsv_lower, dtype=np.uint8)
    upper = np.array(hsv_upper, dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    # 形态学开运算：去小噪点
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 按最小面积过滤噪声
    candidates = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not candidates:
        return {"detected": False, "u": 0.0, "v": 0.0, "area": 0.0}

    # 多候选：选面积最大者为主目标
    c = max(candidates, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] != 0:
        u = M["m10"] / M["m00"]
        v = M["m01"] / M["m00"]
    else:
        x, y, w, h = cv2.boundingRect(c)
        u = x + w / 2.0
        v = y + h / 2.0

    area = float(cv2.contourArea(c))
    return {"detected": True, "u": float(u), "v": float(v), "area": area}


def detect_from_file(image_path, hsv_lower, hsv_upper, min_area):
    """供发布节点调用：文件存在则检测，否则返回 detected=False。"""
    if not os.path.exists(image_path):
        return {"detected": False, "u": 0.0, "v": 0.0, "area": 0.0,
                "error": "image not found: " + image_path}
    return detect_target(image_path, hsv_lower, hsv_upper, min_area)
