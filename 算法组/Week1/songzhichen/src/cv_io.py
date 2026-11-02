# -*- coding: utf-8 -*-
"""cv_io.py —— 能处理中文路径的 OpenCV 读写函数。

用法：把本文件放在 src/ 旁边，脚本里这样用

    from cv_io import imread_u, imwrite_u

    img = imread_u("data/images/blue_box.jpg")
    if img is None:
        raise FileNotFoundError("读不到图片")
    imwrite_u("outputs/task1/gray.png", gray)

背景：Windows 上 cv2.imread / cv2.imwrite 遇到含中文的路径会静默失败
（imwrite 返回 False、imread 返回 None），不抛异常。任务书要求的文件夹名
W1_姓名_视觉算法 带中文，所以要么把项目放在纯英文路径，要么用这里的封装。
"""
import os

import cv2
import numpy as np

__all__ = ["imread_u", "imwrite_u"]


def imread_u(path, flags=cv2.IMREAD_COLOR):
    """读图，支持中文路径。失败返回 None，与 cv2.imread 行为一致。"""
    if not os.path.exists(path):
        return None
    try:
        buf = np.fromfile(path, dtype=np.uint8)
    except Exception:
        return None
    if buf.size == 0:
        return None
    return cv2.imdecode(buf, flags)


def imwrite_u(path, img, ext=None):
    """写图，支持中文路径。返回 True/False，与 cv2.imwrite 行为一致。"""
    if ext is None:
        ext = os.path.splitext(path)[1] or ".png"
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        return False
    try:
        buf.tofile(path)      # tofile 走文件流，不受路径编码影响
    except Exception:
        return False
    return True
