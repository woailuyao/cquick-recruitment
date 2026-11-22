# -*- coding: utf-8 -*-
"""取景判据用到的纯像素指标（只依赖 cv2 / numpy，不加载模型，便于离线回归）。

背景：
    framing_check 早先的绿茵判据是 RGB 绝对差值——
        (g > r + 12) & (g > b + 12) & (g > 40)
    它是照着室外强光天然草写的。室内暗光下的深绿人造草不满足：
    实测 19:18 那帧下半部 G 中位数只有 21、95 分位 46，`g > 40` 一关就砍掉 92% 的像素；
    (G-B) 中位数只有 9，也过不去 `g > b + 12`。三条同时成立只剩 1.52%，
    于是「镜头正对完整半场」也被判成取景不合格。

    改用 HSV：色相圈定绿色，饱和度排除白线（白线 S 很低），明度只留一个很低的
    地板值挡纯黑噪声，不再依赖绝对亮度。同一帧实测 59.5%，白线仍被正确排除。

用法：
    from tools.frame_metrics import grass_ratio, is_gray
"""

from __future__ import annotations

import cv2
import numpy as np

# 草皮判据：H 30~100（绿到青绿，覆盖偏冷的人造草）、S>40（排除白线与灰面）、V>15（排除死黑）
GRASS_HUE_LO = 30
GRASS_HUE_HI = 100
GRASS_SAT_MIN = 40
GRASS_VAL_MIN = 15


def grass_ratio(lower_bgr: np.ndarray) -> float:
    """下半部画面里「草皮像素」的占比。"""
    hsv = cv2.cvtColor(lower_bgr, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0].astype(int)
    s = hsv[:, :, 1].astype(int)
    v = hsv[:, :, 2].astype(int)
    mask = ((h >= GRASS_HUE_LO) & (h <= GRASS_HUE_HI) & (s > GRASS_SAT_MIN) & (v > GRASS_VAL_MIN))
    return float(mask.mean())


def is_gray(frame_bgr: np.ndarray, tol: float = 2.0, sat_min: float = 15.0) -> bool:
    """帧几乎「没有颜色」：灰度采集，或过曝到饱和度被压平。

    这类帧上任何颜色判据都会归零，会被误读成「没对着草皮」；
    采集格式不对（Y8 / 单通道复制成三通道）或曝光爆掉时先在这里拦一道。
    实测：frame_115929 / frame_exposed 的中位饱和度只有 8 / 9（过曝），
    而正常暗光场地的中位饱和度是 182。
    """
    b = frame_bgr[:, :, 0].astype(int)
    g = frame_bgr[:, :, 1].astype(int)
    r = frame_bgr[:, :, 2].astype(int)
    near_neutral = np.mean(np.abs(r - b)) < tol and np.mean(np.abs(g - b)) < tol
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    flat_chroma = float(np.median(hsv[:, :, 1])) < sat_min
    return bool(near_neutral or flat_chroma)
