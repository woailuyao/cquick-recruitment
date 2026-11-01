"""兼容中文路径的图片读写，以及保存后回读检查。"""
from pathlib import Path

import cv2
import numpy as np


def read_image(path, flags=cv2.IMREAD_COLOR) -> np.ndarray:
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Image does not exist: {path}")
    data = np.fromfile(path, dtype=np.uint8)
    image = cv2.imdecode(data, flags) if data.size else None
    if image is None or image.size == 0:
        raise ValueError(f"Cannot decode image: {path}")
    return image


def save_image(path, image: np.ndarray) -> None:
    path = Path(path)
    if not isinstance(image, np.ndarray) or image.size == 0:
        raise ValueError("Cannot save an empty image")
    path.parent.mkdir(parents=True, exist_ok=True)
    success, encoded = cv2.imencode(path.suffix, image)
    if not success:
        raise OSError(f"Cannot encode image: {path}")
    encoded.tofile(path)
    restored = read_image(path, cv2.IMREAD_UNCHANGED)
    if restored.shape != image.shape:
        raise OSError(f"Saved image has changed shape: {path}")
    # PNG 是无损格式，应逐像素一致；JPEG 等有损格式只检查能读取且尺寸一致。
    if path.suffix.lower() == ".png" and not np.array_equal(restored, image):
        raise OSError(f"PNG roundtrip changed pixels: {path}")
