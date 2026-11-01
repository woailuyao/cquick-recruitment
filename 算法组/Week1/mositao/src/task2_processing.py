from pathlib import Path
import cv2
import numpy as np

from io_utils import imread, imwrite

BASE = Path(__file__).resolve().parents[1]
IMG_DIR = BASE / "data" / "images"
OUT_DIR = BASE / "outputs" / "task2"
OUT_DIR.mkdir(parents=True, exist_ok=True)

IMAGE_NAME = "02.JPG"  # 换成你想分析的那张
MAX_SIDE = 800  # 长边缩到 800，处理快、图也看得清

img = imread(str(IMG_DIR / IMAGE_NAME))
if img is None:
    raise SystemExit(f"读取失败: {IMG_DIR / IMAGE_NAME}")

# 大图先缩小，避免 1300 万像素拖慢 Canny
h, w = img.shape[:2]
scale = MAX_SIDE / max(h, w)
if scale < 1:
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
print("处理尺寸:", img.shape)


def save(name, out_img):
    """保存并打印白色像素占比，方便对比阈值效果"""
    path = OUT_DIR / f"{name}.png"
    ok = imwrite(str(path), out_img)
    white_ratio = float((out_img > 127).mean() * 100)
    print(f"  {name:22s} 保存={ok}  白色占比={white_ratio:5.1f}%")
    return ok


# ---------- 1. 灰度 + 三种滤波 ----------
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gauss = cv2.GaussianBlur(gray, (5, 5), 0)  # 高斯：权重随距离衰减
mean_f = cv2.blur(gray, (5, 5))  # 均值：邻域等权重
median = cv2.medianBlur(gray, 5)  # 中值：专治椒盐噪声

save("01_gray", gray)
save("02_gauss_5x5", gauss)
save("03_mean_5x5", mean_f)
save("04_median_5", median)

# ---------- 2. 阈值：固定 + Otsu + 自适应 ----------
_, th_80 = cv2.threshold(gauss, 80, 255, cv2.THRESH_BINARY)
_, th_180 = cv2.threshold(gauss, 180, 255, cv2.THRESH_BINARY)
_, th_otsu = cv2.threshold(gauss, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
adaptive = cv2.adaptiveThreshold(gauss, 255,
                                 cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY, 31, 5)

save("05_thresh_80", th_80)
save("06_thresh_180", th_180)
save("07_otsu", th_otsu)
save("08_adaptive", adaptive)

# ---------- 3. Canny：两组阈值 ----------
edge_lo = cv2.Canny(gauss, 30, 100)  # 低阈值：边缘多，含噪点
edge_hi = cv2.Canny(gauss, 100, 200)  # 高阈值：边缘少，但可能断

save("09_canny_30_100", edge_lo)
save("10_canny_100_200", edge_hi)

# ---------- 4. 形态学：腐蚀/膨胀/开/闭 ----------
k3 = np.ones((3, 3), np.uint8)
k7 = np.ones((7, 7), np.uint8)

eroded = cv2.erode(th_otsu, k3, iterations=1)
dilated = cv2.dilate(th_otsu, k3, iterations=1)
opened = cv2.morphologyEx(th_otsu, cv2.MORPH_OPEN, k7)  # 先腐蚀后膨胀：去小白点
closed = cv2.morphologyEx(th_otsu, cv2.MORPH_CLOSE, k7)  # 先膨胀后腐蚀：补小黑洞

save("11_erode_3x3", eroded)
save("12_dilate_3x3", dilated)
save("13_open_7x7", opened)
save("14_close_7x7", closed)

print("\n完成，结果在:", OUT_DIR)

