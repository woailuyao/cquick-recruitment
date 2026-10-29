"""HSV 颜色分割 + 轮廓筛选，找白色足球的中心点。

参数与 src/task1_target.py 完全一致，改一处要两边同步改，
否则任务1的截图和任务3的消息对不上。
"""
import cv2 as cv
import numpy as np

HSV_LO = (0, 0, 120)      # 白色：S 低、V 高；V 下限放低才够得着球底背光那一圈
HSV_HI = (180, 80, 255)
AREA_MIN = 5000           # 最小轮廓面积（像素平方）
OPEN_K = 21               # 开运算核，吃掉草地白线
CLOSE_K = 9               # 闭运算核，补球面上黑色五边形造成的洞
AR_LO, AR_HI = 0.6, 1.7   # 长宽比，挡掉细长白线


def load(path):
    """读图。不能用 cv.imread——它对非 ASCII 路径静默返回 None。"""
    img = cv.imdecode(np.fromfile(str(path), np.uint8), cv.IMREAD_COLOR)
    return img if img is not None and img.size else None


def detect(img):
    """返回 (u, v, area)；没找到目标返回 None。

    u/v 是像素坐标，原点在左上角，u 向右、v 向下。
    """
    if img is None or img.size == 0:
        return None

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, HSV_LO, HSV_HI)
    mask = cv.morphologyEx(
        mask, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_ELLIPSE, (OPEN_K, OPEN_K))
    )
    mask = cv.morphologyEx(
        mask, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K))
    )

    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cand = []
    for c in cnts:
        area = cv.contourArea(c)
        if area < AREA_MIN:
            continue
        _, _, bw, bh = cv.boundingRect(c)
        if not (AR_LO <= bw / bh <= AR_HI):
            continue
        cand.append((area, c))

    if not cand:
        return None

    cand.sort(key=lambda ac: ac[0], reverse=True)   # 多个候选取面积最大的
    area, cnt = cand[0]

    m = cv.moments(cnt)
    if m["m00"] > 0:                                # m00 为 0 时退化成外接矩形中心
        u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
    else:
        x, y, bw, bh = cv.boundingRect(cnt)
        u, v = x + bw / 2, y + bh / 2

    return u, v, area
