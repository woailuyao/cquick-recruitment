"""Image file I/O that also supports Chinese paths on Windows."""

from pathlib import Path

import cv2 as cv
import numpy as np


def read_image(path):
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"图片不存在：{path}")
    image = cv.imdecode(np.fromfile(str(path), dtype=np.uint8), cv.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"图片无法解码：{path}")
    return image


def save_image(path, pixels, jpeg_quality=90):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    params = ([cv.IMWRITE_JPEG_QUALITY, jpeg_quality]
              if path.suffix.lower() in (".jpg", ".jpeg")
              else [cv.IMWRITE_PNG_COMPRESSION, 9])
    ok, data = cv.imencode(path.suffix, pixels, params)
    if not ok:
        raise OSError(f"图片保存失败：{path}")
    data.tofile(str(path))
