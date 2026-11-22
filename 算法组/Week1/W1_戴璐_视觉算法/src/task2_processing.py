import cv2 as cv
import os
import numpy as np


#设置路径和读取图片
img = cv.imread('data/images/task2_processing_image.jpg')

if img is None:
    sys.exit("无法读取图片，请检查路径是否正确！")
else: 
    
    print("=" * 30)
    print(f"图片读取成功！")
    print('=' * 30)

    save_dir = 'outputs/task2'
    os.makedirs(save_dir, exist_ok=True)
  

#1.灰度转换
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imwrite(f'{save_dir}/01_gray.jpg', gray)
print('=' * 30)
print("灰度图已保存(两组参数对比)")
print('=' * 30)



#2.高斯滤波（去噪/平滑）
blur_5 = cv.GaussianBlur(gray, (5, 5), 0)
blur_15 = cv.GaussianBlur(gray, (15, 15), 0)
cv.imwrite(f'{save_dir}/02_blur_5x5.jpg', blur_5)
cv.imwrite(f'{save_dir}/03_blur_15x15.jpg', blur_15)
print('=' * 30)
print("高斯滤波图片已保存(两组参数对比)")
print('=' * 30)


#3.阈值分割
_, thresh_100 = cv.threshold(blur_5, 100, 255, cv.THRESH_BINARY)
_, thresh_150 = cv.threshold(blur_5, 150, 255, cv.THRESH_BINARY)
cv.imwrite(f'{save_dir}/04_thresh_100.jpg', thresh_100)
cv.imwrite(f'{save_dir}/05_thresh_150.jpg', thresh_150)
print('=' * 30)
print("阈值分割图已保存(两组参数对比)")
print('=' * 30)



#4.Canny边缘检测
edges_strict = cv.Canny(blur_5, 100, 200)
edges_loose = cv.Canny(blur_5, 30, 80)
cv.imwrite(f'{save_dir}/06_canny_strict.jpg', edges_strict)
cv.imwrite(f'{save_dir}/07_canny_loose.jpg', edges_loose)
print('=' * 30)
print("Canny边缘图已保存(两组参数对比)")
print('=' * 30)


#5.形态学处理（开运算 vs 闭运算）
kernel = np.ones((5,5), np.uint8)

#开运算
opening = cv.morphologyEx(thresh_100, cv.MORPH_OPEN, kernel)
#闭运算
closing = cv.morphologyEx(thresh_100, cv.MORPH_CLOSE, kernel)
cv.imwrite(f'{save_dir}/08_morph_open.jpg', opening)
cv.imwrite(f'{save_dir}/09_morph_close.jpg', closing)
print('=' * 30)
print("形态学处理图已保存(开运算/闭运算)")
print('=' * 30)






