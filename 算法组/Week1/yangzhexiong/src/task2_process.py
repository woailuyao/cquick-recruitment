import cv2
import os
import numpy as np
input_path= '../data/images/test3.jpg'
output_folder='../outputs/task2'
os.makedirs(output_folder,exist_ok=True)
img=cv2.imread(input_path)
if img is None:
    print(f"读取失败 请检查{input_path}是否存在")
    exit()
cv2.imwrite(f'{output_folder}/0_original.jpg',img)
gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
cv2.imwrite(f'{output_folder}/1_gray.jpg',gray)
print("已保存灰度图")
blur_small=cv2.GaussianBlur(gray,(5,5),0)
cv2.imwrite(f'{output_folder}/2_blur_small_5x5.jpg',blur_small)
blur_large=cv2.GaussianBlur(gray,(15,15),0)
cv2.imwrite(f'{output_folder}/3_blur_large_15xs15.jpg',blur_large)
print("已保存高斯滤波对比图")
_, thresh_low=cv2.threshold(blur_small,80,255,cv2.THRESH_BINARY)
cv2.imwrite(f'{output_folder}/4_thresh_low_80.jpg',thresh_low)
_, thresh_high=cv2.threshold(blur_small,160,255,cv2.THRESH_BINARY)
cv2.imwrite(f'{output_folder}/5_thresh_high.jpg',thresh_high)
print("已保存阈值分割对比图")
canny=cv2.Canny(blur_small,50,150)
cv2.imwrite(f'{output_folder}/6_canny.jpg',canny)
print("已保存Canny边缘图")
kernel= np.ones((5,5),np.uint8)
opening = cv2.morphologyEx(thresh_low,cv2.MORPH_OPEN,kernel)
cv2.imwrite(f'{output_folder}/7_morphology_open.jpg',opening)
print("已保存形态学（开运算）处理图")
print("\n任务二所有图片处理完毕！请去 outputs/task2 文件夹查看")