"""OpenCV 读写封装：支持含中文/空格的路径

背景：OpenCV 5.0 在 Windows 上调用 imread/imwrite 时，内部按窄字符转换路径，
遇到中文目录名会直接失败（返回 None / False），而不会抛出异常。
绕开办法：用 np.fromfile + cv2.imdecode 读，用 cv2.imencode + tofile 写。

本文件被 src/task1.py、src/task2.py 共用（任务 3 的 ROS2 节点沿用自身路径处理）。
"""
from pathlib import Path

import cv2
import numpy as np


def imread(path, flags=cv2.IMREAD_COLOR):
    """读取图片，兼容中文路径。失败返回 None，不会抛异常。"""
    p = str(path)
    if p.isascii():
        return cv2.imread(p, flags)
    try:
        data = np.fromfile(p, dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, flags)


def imwrite(path, img, params=None):
    """写入图片，兼容中文路径。返回 True/False。"""
    if img is None:
        return False
    p = str(path)
    params = params or []
    if p.isascii():
        return bool(cv2.imwrite(p, img, params))
    ext = Path(p).suffix or ".png"
    ok, buf = cv2.imencode(ext, img, params)
    if not ok:
        return False
    buf.tofile(p)
    return True
