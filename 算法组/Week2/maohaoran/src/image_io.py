"""带中文路径支持的图片读写。

问题：Windows 下 cv2.imread / cv2.imwrite 只接受 ASCII 路径，
      项目目录名含中文时会静默失败（imread 返回 None，imwrite 返回 False）。
解决：先用 numpy 按二进制读入整个文件，再交给 cv2.imdecode 解码；
      写出时先 cv2.imencode 编码，再用 tofile 写回文件。
"""

from pathlib import Path

import cv2
import numpy as np


def imread_unicode(path, flags=cv2.IMREAD_COLOR):
    """读取图片，支持中文路径；失败返回 None。"""
    data = np.fromfile(str(path), dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, flags)


def imwrite_unicode(path, image):
    """保存图片，支持中文路径；成功返回 True。"""
    path = Path(path)
    suffix = path.suffix if path.suffix else ".png"
    ok, buffer = cv2.imencode(suffix, image)
    if not ok:
        return False
    buffer.tofile(str(path))
    return True
