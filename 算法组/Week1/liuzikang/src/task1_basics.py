import os
import sys
import cv2
import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(ROOT, "data", "images")
OUT_DIR = os.path.join(ROOT, "outputs", "task1")
os.makedirs(OUT_DIR, exist_ok=True)

def read_image(path, flags=cv2.IMREAD_COLOR):
    """兼容中文路径的读图：先读文件字节，再让 OpenCV 解码。"""
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, flags)

def save_image(path, image):
    """兼容中文路径的保存：先编码成 PNG 字节，再写入文件。"""
    ok, data = cv2.imencode(".png", image)
    if not ok:
        raise IOError("保存失败：" + path)
    data.tofile(path)

for name in ["cup_clean", "cup_top"]:
    image_path = os.path.join(DATA_DIR, name + ".jpg")

    image = read_image(image_path)
    if image is None:
        print("读取失败：", image_path)
        continue

    height, width, channels = image.shape
    print(name, "宽=", width, "高=", height, "通道=", channels)

    new_width = 640
    new_height = int(height * new_width / width)
    resized = cv2.resize(image, (new_width, new_height))

    x1 = int(width * 0.10)
    x2 = int(width * 0.90)
    y1 = int(height * 0.05)
    y2 = int(height * 0.95)
    crop = image[y1:y2, x1:x2]
    print("裁剪坐标：", (x1, y1, x2, y2), "裁剪尺寸：", crop.shape[1], "x", crop.shape[0])

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    save_image(os.path.join(OUT_DIR, name + "_resized.png"), resized)
    save_image(os.path.join(OUT_DIR, name + "_crop.png"), crop)
    save_image(os.path.join(OUT_DIR, name + "_gray.png"), gray)

    check = read_image(
        os.path.join(OUT_DIR, name + "_gray.png"),
        cv2.IMREAD_GRAYSCALE,
    )
    print("回读成功：", check is not None, "尺寸：", check.shape)