import cv2
import numpy as np
import os

# 准备工作
os.makedirs('W1/outputs/task2', exist_ok=True)
img = cv2.imread('W1/data/images/apple1.jpg')

if img is None:
    print("读取失败")
    exit()

# 1. 转灰度
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imwrite('W1/outputs/task2/1_gray.jpg', gray)

# 2. 高斯滤波对比
blur_small = cv2.GaussianBlur(gray, (5, 5), 0)
blur_large = cv2.GaussianBlur(gray, (25, 25), 0)
cv2.imwrite('W1/outputs/task2/2_blur_5x5.jpg', blur_small)
cv2.imwrite('W1/outputs/task2/2_blur_25x25.jpg', blur_large)

# 3. 阈值分割对比
_, thresh_low = cv2.threshold(blur_large, 100, 255, cv2.THRESH_BINARY)
_, thresh_high = cv2.threshold(blur_large, 175, 255, cv2.THRESH_BINARY)
cv2.imwrite('W1/outputs/task2/3_thresh_100.jpg', thresh_low)
cv2.imwrite('W1/outputs/task2/3_thresh_175.jpg', thresh_high)

# 4. Canny边缘检测
edges = cv2.Canny(blur_large, 10, 30)
cv2.imwrite('W1/outputs/task2/4_canny_10_30.jpg', edges)

# 5. 形态学处理
kernel = np.ones((20, 20), np.uint8)
opened = cv2.morphologyEx(thresh_low, cv2.MORPH_OPEN, kernel)
closed = cv2.morphologyEx(thresh_high, cv2.MORPH_CLOSE, kernel)
cv2.imwrite('W1/outputs/task2/5_morph_open.jpg', opened)
cv2.imwrite('W1/outputs/task2/5_morph_close.jpg', closed)

print("处理完成")