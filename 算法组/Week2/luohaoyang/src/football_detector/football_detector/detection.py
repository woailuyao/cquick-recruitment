from pathlib import Path

import cv2
import numpy as np


def read_image(path):
    data = np.fromfile(str(path), dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def refine_circle(gray, x, y, w, h):
    height, width = gray.shape[:2]
    pad = 40
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(width, x + w + pad)
    y1 = min(height, y + h + pad)

    roi = cv2.GaussianBlur(gray[y0:y1, x0:x1], (9, 9), 2)
    circles = cv2.HoughCircles(
        roi,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=100,
        param1=100,
        param2=40,
        minRadius=60,
        maxRadius=200,
    )
    if circles is None or len(circles[0]) == 0:
        return None

    points = []
    for cx, cy, radius in np.round(circles[0]).astype(int):
        points.append((int(cx + x0), int(cy + y0), int(radius)))

    bbox_cx = x + w / 2
    bbox_cy = y + h / 2
    return min(
        points,
        key=lambda c: (c[0] - bbox_cx) ** 2 + (c[1] - bbox_cy) ** 2,
    )


def detect_football(path, min_area=500):
    """
    Detect the football in one image.

    Returns None when no target exists.  Otherwise returns:
        {"detected": True, "u": float, "v": float, "area": float}
    """
    img = read_image(path)
    if img is None:
        return None

    img = cv2.resize(img, None, fx=0.3, fy=0.3)
    height, width = img.shape[:2]

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 120], dtype=np.uint8)
    upper_white = np.array([180, 45, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_white, upper_white)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
    )

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= min_area:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
            continue

        aspect = w / max(h, 1)
        if not (0.5 <= aspect <= 2.0):
            continue
        candidates.append(contour)

    if not candidates:
        return None

    target = max(candidates, key=cv2.contourArea)
    area = float(cv2.contourArea(target))
    x, y, w, h = cv2.boundingRect(target)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    circle = refine_circle(gray, x, y, w, h)

    if circle is not None:
        u, v, _radius = circle
    else:
        moments = cv2.moments(target)
        if moments["m00"] == 0:
            return None
        u = int(moments["m10"] / moments["m00"])
        v = int(moments["m01"] / moments["m00"])

    return {
        "detected": True,
        "u": float(u),
        "v": float(v),
        "area": area,
    }