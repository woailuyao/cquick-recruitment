import os
import sys
import cv2
import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "data", "images", "red_can.jpg")
OUT = os.path.join(ROOT, "outputs", "task2")

def read_image(path):
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)

def save_image(path, image):
    ok, data = cv2.imencode(".png", image)
    if not ok:
        raise IOError("保存失败：" + path)
    data.tofile(path)

    back = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if back is None:
        raise IOError("回读失败：" + path)

def main():
    os.makedirs(OUT, exist_ok=True)

    image = read_image(SRC)
    if image is None:
        raise FileNotFoundError("读取失败：" + SRC)

    image = image[250:1200, 200:880].copy()
    h, w = image.shape[:2]
    print("处理尺寸：", w, "x", h)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    save_image(os.path.join(OUT, "00_original.png"), image)
    save_image(os.path.join(OUT, "01_gray.png"), gray)

    mean5 = cv2.blur(gray, (5, 5))
    gaussian5 = cv2.GaussianBlur(gray, (5, 5), 0)
    gaussian15 = cv2.GaussianBlur(gray, (15, 15), 0)
    median5 = cv2.medianBlur(gray, 5)

    save_image(os.path.join(OUT, "02_mean_5x5.png"), mean5)
    save_image(os.path.join(OUT, "03_gaussian_5x5.png"), gaussian5)
    save_image(os.path.join(OUT, "04_gaussian_15x15.png"), gaussian15)
    save_image(os.path.join(OUT, "05_median_5.png"), median5)

    for t in (50, 80):
        _, mask = cv2.threshold(gaussian5, t, 255, cv2.THRESH_BINARY_INV)
        save_image(os.path.join(OUT, f"06_threshold_{t}.png"), mask)
        print(f"固定阈值 {t}: 白色占比 = {np.count_nonzero(mask) / mask.size:.2%}")

    adaptive = cv2.adaptiveThreshold(
        gaussian5, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 5
    )
    save_image(os.path.join(OUT, "07_adaptive_31_c5.png"), adaptive)
    print(f"自适应阈值: 白色占比 = {np.count_nonzero(adaptive) / adaptive.size:.2%}")

    for low, high in ((50, 150), (100, 200)):
        edges = cv2.Canny(gaussian5, low, high, L2gradient=True)
        save_image(os.path.join(OUT, f"08_canny_{low}_{high}.png"), edges)
        print(f"Canny {low}/{high}: 边缘像素 = {np.count_nonzero(edges)}")

    _, base_mask = cv2.threshold(gaussian5, 50, 255, cv2.THRESH_BINARY_INV)
    k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    k15 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

    opened = cv2.morphologyEx(base_mask, cv2.MORPH_OPEN, k5)
    closed = cv2.morphologyEx(base_mask, cv2.MORPH_CLOSE, k15)
    opened_closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, k15)

    save_image(os.path.join(OUT, "09_morph_base.png"), base_mask)
    save_image(os.path.join(OUT, "10_open_5x5.png"), opened)
    save_image(os.path.join(OUT, "11_close_15x15.png"), closed)
    save_image(os.path.join(OUT, "12_open5_close15.png"), opened_closed)

    for name, mask in (("原始掩膜", base_mask), ("开运算", opened),
                       ("闭运算", closed), ("先开后闭", opened_closed)):
        count, _ = cv2.connectedComponents(mask)
        print(f"{name}: 连通域 {count - 1} 个")

    print("完成，结果在：", OUT)

if __name__ == "__main__":
    main()