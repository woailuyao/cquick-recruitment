"""跨平台读写图片的小工具。

Windows 上 cv2.imread / cv2.imwrite 遇到中文路径会直接失败（返回 None / False），
所以统一走 np.fromfile + cv2.imdecode / cv2.imencode + tofile。
"""

import os

import cv2
import numpy as np


def imread(path: str):
    """读图，支持中文路径；失败返回 None。"""
    try:
        data = np.fromfile(path, dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    return img


def imwrite(path: str, img) -> bool:
    """写图，支持中文路径。"""
    ext = os.path.splitext(path)[1].lower() or ".jpg"
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        return False
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    buf.tofile(path)
    return True


def list_images(folder: str):
    """列出文件夹里所有图片（不递归），按文件名排序。"""
    exts = (".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff")
    if not os.path.isdir(folder):
        return []
    names = [n for n in os.listdir(folder) if n.lower().endswith(exts)]
    return [os.path.join(folder, n) for n in sorted(names)]
