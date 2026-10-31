import cv2
import os
import numpy as np

os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

img = cv2.imread("data/images/pen.jpg")
#输出灰度图
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imwrite("outputs/task2/gray2.jpg", gray)
#高斯滤波结果
blur1 = cv2.GaussianBlur(gray, (5, 5), 0)
cv2.imwrite("outputs/task2/blur_5x5.jpg", blur1)

blur2 = cv2.GaussianBlur(gray, (15, 15), 0)
cv2.imwrite("outputs/task2/blur_15x15.jpg", blur2)

#阈值分割结果
_, thresh1 = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
cv2.imwrite("outputs/task2/thresh1.jpg", thresh1)

_, thresh2 = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
cv2.imwrite("outputs/task2/thresh2.jpg", thresh2)

#Canny 边缘图
edges = cv2.Canny(blur1, 50, 150)
cv2.imwrite("outputs/task2/canny.jpg", edges)

edges2 = cv2.Canny(blur1, 100, 200)
cv2.imwrite("outputs/task2/canny2.jpg", edges2)

#一次形态学处理
kernel = np.ones((5, 5), np.uint8)
opening = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel)
cv2.imwrite("outputs/task2/opening.jpg", opening)


