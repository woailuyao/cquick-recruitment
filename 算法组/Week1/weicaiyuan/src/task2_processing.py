import cv2
import numpy as np
import os

img_path = "data/images/picture1.jpg" # 替换为你的原图路径
img = cv2.imread(img_path)
if img is None:
    raise FileNotFoundError(f"读不到图片：{img_path}")

out_dir = "outputs/task2"
os.makedirs(out_dir, exist_ok=True)

# 1. 灰度图
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imwrite(os.path.join(out_dir, "01_gray.jpg"), gray)

# 2. 高斯滤波：3x3 vs 7x7
blur_3 = cv2.GaussianBlur(gray, (3, 3), 0)
blur_7 = cv2.GaussianBlur(gray, (7, 7), 0)
cv2.imwrite(os.path.join(out_dir, "02_blur_3x3.jpg"), blur_3)
cv2.imwrite(os.path.join(out_dir, "03_blur_7x7.jpg"), blur_7)

# 3. 阈值分割：正向80 vs 正向150
_, th_bin_80 = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
_, th_bin_150 = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
cv2.imwrite(os.path.join(out_dir, "04_thresh_bin_80.jpg"), th_bin_80)
cv2.imwrite(os.path.join(out_dir, "05_thresh_bin_150.jpg"), th_bin_150)

# 4. 反向阈值：提取灰色身体（用200作为阈值）
_, th_inv_200 = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
cv2.imwrite(os.path.join(out_dir, "06_thresh_inv_200.jpg"), th_inv_200)

# 5. Canny 边缘：50/150 vs 100/200
canny_50 = cv2.Canny(gray, 50, 150)
canny_100 = cv2.Canny(gray, 100, 200)
cv2.imwrite(os.path.join(out_dir, "07_canny_50_150.jpg"), canny_50)
cv2.imwrite(os.path.join(out_dir, "08_canny_100_200.jpg"), canny_100)

# 6. 形态学：对反向阈值图（图6）做开运算和闭运算
kernel = np.ones((5, 5), np.uint8)
open_res = cv2.morphologyEx(th_inv_200, cv2.MORPH_OPEN, kernel)
close_res = cv2.morphologyEx(th_inv_200, cv2.MORPH_CLOSE, kernel)

# --- 为了帮你一眼看懂，给图片加上文字标注 ---
# 仅用于方便观察，存图时加了文字
def add_text(img_to_text, text):
    img_color = cv2.cvtColor(img_to_text, cv2.COLOR_GRAY2BGR)
    cv2.putText(img_color, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    return img_color

cv2.imwrite(os.path.join(out_dir, "09_morph_open_5x5.jpg"), add_text(open_res, "Open 5x5"))
cv2.imwrite(os.path.join(out_dir, "10_morph_close_5x5.jpg"), add_text(close_res, "Close 5x5"))

print("全部完成！请查看 outputs/task2/ 文件夹，图片已按编号和文字命名。")