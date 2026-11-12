
# detector_core.py —— 只做检测，一行都不 import rclpy
import cv2
import numpy as np

# 橙色（篮球）的 HSV 范围。OpenCV 的 H 是 0~179，不是 0~360
LOWER = np.array([5, 90, 70], np.uint8)
UPPER = np.array([25, 255, 255], np.uint8)
MIN_AREA = 1500


def detect_target(frame, lower=LOWER, upper=UPPER, min_area=MIN_AREA):
    """返回 dict：found / u / v / area / box / contour / mask"""
    lost = {'found': False, 'u': 0.0, 'v': 0.0, 'area': 0.0,
            'box': None, 'contour': None, 'mask': None}

    if frame is None:
        return lost

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return dict(lost, mask=mask)

    cands = []
    for c in contours:
        a = cv2.contourArea(c)
        if a < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if not (0.6 <= w / max(h, 1) <= 1.8):
            continue
        cands.append((a, c))

    if not cands:
        return dict(lost, mask=mask)

    area, best = max(cands, key=lambda t: t[0])
    x, y, w, h = cv2.boundingRect(best)

    m = cv2.moments(best)
    if m['m00'] == 0:
        u, v = x + w / 2, y + h / 2
    else:
        u, v = m['m10'] / m['m00'], m['m01'] / m['m00']

    return {'found': True, 'u': float(u), 'v': float(v), 'area': float(area),
            'box': (x, y, w, h), 'contour': best, 'mask': mask}