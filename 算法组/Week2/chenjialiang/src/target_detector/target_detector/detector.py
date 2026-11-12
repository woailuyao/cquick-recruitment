"""和 detect.py 同一套检测逻辑，只返回坐标，不画图不存盘。"""

import cv2
import numpy as np

LOWER = np.array([0, 0, 160])
UPPER = np.array([179, 255, 255])

MIN_RATIO = 0.002
K_OPEN = (5, 5)
K_CLOSE = (15, 15)


def detect_image(img):
    """返回 {"u":.., "v":.., "area":..}，未检测到返回 None"""
    if img is None:
        return None

    he, we = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER, UPPER)

    mask_pro = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones(K_OPEN, np.uint8))
    mask_pro = cv2.morphologyEx(mask_pro, cv2.MORPH_CLOSE, np.ones(K_CLOSE, np.uint8))

    contours, _ = cv2.findContours(mask_pro, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_areas = MIN_RATIO * he * we
    kept = [c for c in contours if cv2.contourArea(c) > min_areas]
    if not kept:
        return None

    cnt = max(kept, key=cv2.contourArea)
    area = cv2.contourArea(cnt)

    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None

    return {
        "u": float(M["m10"] / M["m00"]),
        "v": float(M["m01"] / M["m00"]),
        "area": float(area),
    }
