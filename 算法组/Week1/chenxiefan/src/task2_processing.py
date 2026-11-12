import cv2 as cv
import numpy as np
import os
import sys

#建立路径、导入图片
input_dir = os.path.join('..', 'data', 'images')
output_dir = os.path.join('..', 'outputs', 'task2')
img_path = os.path.join(input_dir, 'SleepingCat.jpg')
img = cv.imread(img_path,1)
cv.imshow('My window',img)
print("导入成功，请按任意键继续")
cv.waitKey(0)
cv.destroyAllWindows()

#生成灰度图
img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imshow('My window',img_gray)
gray_path = os.path.join(output_dir, "out_gray.png")
result = cv.imwrite(gray_path,img_gray,[cv.IMWRITE_PNG_COMPRESSION, 3])
print("灰度图生成成功，请按任意键继续")
cv.waitKey(0)
cv.destroyAllWindows()

#在灰度图基础上进行高斯滤波
img_blurred = cv.GaussianBlur(img_gray, (5,5), 0)
cv.imshow('My window',img_blurred)
blurred_path = os.path.join(output_dir, "out_blurred.png")
result = cv.imwrite(blurred_path,img_blurred,[cv.IMWRITE_PNG_COMPRESSION, 3])
print("高斯滤波成功，请按任意键继续")
cv.waitKey(0)
cv.destroyAllWindows()

#在高斯滤波图基础上进行阈值分割
ret, img_thresh = cv.threshold(img_blurred, 127, 255, cv.THRESH_BINARY)
cv.imshow('My window',img_thresh)
thresh_path = os.path.join(output_dir, "out_thresh.png")
result = cv.imwrite(thresh_path,img_thresh,[cv.IMWRITE_PNG_COMPRESSION, 3])
print("阈值分割成功，请按任意键继续")
cv.waitKey(0)
cv.destroyAllWindows()

#在高斯滤波图基础上进行Canny边缘检测
img_edges = cv.Canny(img_blurred, 150, 200)
cv.imshow('My window',img_edges)
edges_path = os.path.join(output_dir, "out_edges.png")
result = cv.imwrite(edges_path,img_edges,[cv.IMWRITE_PNG_COMPRESSION, 3])
print("Canny边缘检测成功，请按任意键继续")
cv.waitKey(0)
cv.destroyAllWindows()

#在黑白图的基础上进行开运算
kernel = np.ones((5, 5), np.uint8) 
img_opening = cv.morphologyEx(img_thresh, cv.MORPH_OPEN, kernel)
cv.imshow('My window',img_opening)
opening_path = os.path.join(output_dir, "out_opening.png")
result = cv.imwrite(opening_path,img_opening,[cv.IMWRITE_PNG_COMPRESSION, 3])
print("开运算成功，请按任意键继续")
cv.waitKey(0)
cv.destroyAllWindows()
