import cv2
import numpy as np
import os

# 自动创建输出文件夹，防止保存时报错
os.makedirs(r'D:\MyCode\outputs\task1', exist_ok=True)

# 读取原图
img = cv2.imread(r'D:\MyCode\data\images\1.jpeg')

# 1. 灰度化：将三通道彩色图转换为单通道灰度图
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 2. HSV 转换：将 BGR 转为 HSV，HSV 更容易根据颜色（H）提取目标
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 3. 阈值分割对比：
# 阈值 A：设为 127，大于 127 的变白(255)，小于的变黑(0)
_, thresh_a = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
# 阈值 B：调高到 200，只有非常亮的部分才会变白，大片区域变黑
_, thresh_b = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

# 将处理结果保存到文件夹，方便考官查看
cv2.imwrite(r'D:\MyCode\outputs\task1\01_original.png', img)
cv2.imwrite(r'D:\MyCode\outputs\task1\02_gray.png', gray)
cv2.imwrite(r'D:\MyCode\outputs\task1\03_hsv.png', hsv)
cv2.imwrite(r'D:\MyCode\outputs\task1\04_thresh_a.png', thresh_a)
cv2.imwrite(r'D:\MyCode\outputs\task1\05_thresh_b.png', thresh_b)

print("处理完成，5张结果图已保存到 outputs 文件夹！")