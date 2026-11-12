import numpy as np
import cv2 as cv

#灰度图
img = cv.imread("data/images/img2.jpg")
if img is None:
    print("图片读取失败")
    exit()

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imwrite("outputs/task2/01_gray.jpg", gray)

#高斯滤波
gauss_3 = cv.GaussianBlur(gray, (3, 3), 0)
gauss_7 = cv.GaussianBlur(gray, (7, 7), 0)
cv.imwrite("outputs/task2/02_gauss_3x3.jpg", gauss_3)
cv.imwrite("outputs/task2/02_gauss_7x7.jpg", gauss_7)

#阈值分割
_, thresh_low  = cv.threshold(gauss_3, 150, 255, cv.THRESH_BINARY)
_, thresh_high = cv.threshold(gauss_3, 180, 255, cv.THRESH_BINARY)
cv.imwrite("outputs/task2/03_thresh_150.jpg", thresh_low)
cv.imwrite("outputs/task2/03_thresh_180.jpg", thresh_high)

#Canny边缘图
canny_low  = cv.Canny(gauss_3, 30, 90)
canny_high = cv.Canny(gauss_3, 80, 200)
cv.imwrite("outputs/task2/04_canny_30_90.jpg", canny_low)
cv.imwrite("outputs/task2/04_canny_80_200.jpg", canny_high)

#形态学处理
kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

#开运算
opened = cv.morphologyEx(thresh_low, cv.MORPH_OPEN, kernel)
cv.imwrite("outputs/task2/05_open_5x5.jpg", opened)

#闭运算
closed = cv.morphologyEx(thresh_low, cv.MORPH_CLOSE, kernel)
cv.imwrite("outputs/task2/05_close_5x5.jpg", closed)

print("任务二处理完成")