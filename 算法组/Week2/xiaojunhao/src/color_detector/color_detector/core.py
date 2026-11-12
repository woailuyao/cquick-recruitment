# -*- coding: utf-8 -*-
"""颜色目标检测的核心逻辑。

这个模块只依赖 OpenCV 和 numpy，不导入 rclpy，
所以可以单独跑、单独测，ROS2 节点只是把它的输出转发出去。
"""

import cv2
import numpy as np

# 橙色球的 HSV 范围。OpenCV 里 H 是 0~179，S/V 是 0~255。
# 数值是先把目标区域的像素采样出来看分布，再留一点余量定的。
# S 下限取 170：03 那张图的木桌背景 S 大概 135，目标在 200 以上，
# 取 170 刚好把背景挡在外面（取 120 的话整幅背景都会被当成目标）。
DEFAULT_LOWER = (5, 170, 80)
DEFAULT_UPPER = (22, 255, 255)

# 最小轮廓面积（像素平方）。1280x960 下目标大约 2 万到 9 万像素，
# 开运算之后剩下的噪点一般在 500 以内，1500 卡在中间。
DEFAULT_MIN_AREA = 1500


class Target(object):
    """一次检测的结果。没检测到目标时不会创建这个对象。"""

    def __init__(self, u, v, area, bbox, contour, candidates):
        self.u = u                  # 中心横坐标（像素）
        self.v = v                  # 中心纵坐标（像素）
        self.area = area            # 轮廓面积（像素平方）
        self.bbox = bbox            # (x, y, w, h)
        self.contour = contour
        self.candidates = candidates  # 过了面积阈值的轮廓个数


def build_mask(bgr, lower=DEFAULT_LOWER, upper=DEFAULT_UPPER, morph=True):
    """BGR -> HSV -> 二值掩膜，可选做一次开运算去掉零星噪点。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    if morph:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def detect(bgr, lower=DEFAULT_LOWER, upper=DEFAULT_UPPER,
           min_area=DEFAULT_MIN_AREA, morph=True):
    """返回 (Target 或 None, mask, 轮廓列表)。

    多个候选时按面积从大到小排序，取最大的那个：
    目标一般比背景里的杂色块大，取最大最稳。
    """
    mask = build_mask(bgr, lower, upper, morph)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 面积太小的直接丢掉，剩下的按面积排序
    valid = [c for c in contours if cv2.contourArea(c) >= min_area]
    valid.sort(key=cv2.contourArea, reverse=True)
    if not valid:
        return None, mask, contours

    contour = valid[0]
    area = float(cv2.contourArea(contour))

    # 用图像矩算质心；如果 m00 是 0（面积退化），退回外接矩形中心
    m = cv2.moments(contour)
    if m["m00"] > 0:
        u = m["m10"] / m["m00"]
        v = m["m01"] / m["m00"]
    else:
        x, y, w, h = cv2.boundingRect(contour)
        u = x + w / 2.0
        v = y + h / 2.0

    target = Target(u, v, area, cv2.boundingRect(contour), contour, len(valid))
    return target, mask, contours


def annotate(bgr, target, color=(0, 255, 0)):
    """把轮廓、外接矩形、中心点和坐标文字画到图上。"""
    out = bgr.copy()
    if target is None:
        cv2.putText(out, "no target", (20, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    1.6, (0, 0, 255), 3, cv2.LINE_AA)
        return out

    cv2.drawContours(out, [target.contour], -1, color, 3)
    x, y, w, h = target.bbox
    cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)

    cu, cv_ = int(round(target.u)), int(round(target.v))
    cv2.drawMarker(out, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
    cv2.circle(out, (cu, cv_), 12, (0, 0, 255), 2)

    text = "u=%d, v=%d" % (cu, cv_)
    sub = "area=%d" % int(target.area)

    # 文字放在中心点右上方，超出画面就往回挪；底下垫一块深色，免得压住球看不清
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
    (sw, _), _ = cv2.getTextSize(sub, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    box_w = max(tw, sw) + 20
    tx = min(max(cu + 24, 8), out.shape[1] - box_w - 8)
    ty = max(cv_ - 24, th + 10)
    cv2.rectangle(out, (tx - 6, ty - th - 8), (tx + box_w, ty + 46), (0, 0, 0), -1)

    cv2.putText(out, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(out, sub, (tx, ty + 38), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 255, 255), 2, cv2.LINE_AA)
    return out
