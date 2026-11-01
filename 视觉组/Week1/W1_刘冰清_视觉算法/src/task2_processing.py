#任务二：基础图像处理对比
#作者：刘冰清
#运行环境： Python 3.x, OpenCV
import cv2
import os
import matplotlib.pyplot as plt
import numpy as np
img_path = 'inputs/img2.png'
output_dir = 'outputs/task2'
os.makedirs(output_dir, exist_ok=True)
img = cv2.imread(img_path) #读取图片
if img is None:
    raise(f"[错误] 无法读取图片: {image_path}, 请检查路径是否正确。")
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #颜色转换
img_gaussian_1 = cv2.GaussianBlur(img_gray, (3, 3), 1)
img_gaussian_2 = cv2.GaussianBlur(img_gray, (15, 15), 5)
_,img_threshold_1 = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
_,img_threshold_2 = cv2.threshold(img_gray, 200, 255, cv2.THRESH_BINARY)
img_canny_1 = cv2.Canny(img_gray, 50, 150)
img_canny_2 = cv2.Canny(img_gray, 30, 100)
kernel_1 = np.ones((3, 3), np.uint8)
kernel_2 = np.ones((5, 5), np.uint8)
img_morphology_1 = cv2.morphologyEx(img_threshold_1, cv2.MORPH_OPEN, kernel_1)
img_morphology_2 = cv2.morphologyEx(img_threshold_2, cv2.MORPH_OPEN, kernel_2)
cv2.imwrite(os.path.join(output_dir, '1_gray.png'), img_gray) #保存图片
cv2.imwrite(os.path.join(output_dir, '2_gaussian_3x3.png'), img_gaussian_1) #保存图片
cv2.imwrite(os.path.join(output_dir, '2_gaussian_15x15.png'), img_gaussian_2) #保存图片
cv2.imwrite(os.path.join(output_dir, '3_threshold_127.png'), img_threshold_1) #保存图片
cv2.imwrite(os.path.join(output_dir, '3_threshold_200.png'), img_threshold_2) #保存图片
cv2.imwrite(os.path.join(output_dir, '4_canny_50_150.png'), img_canny_1) #保存图片
cv2.imwrite(os.path.join(output_dir, '4_canny_30_100.png'), img_canny_2) #保存图片
cv2.imwrite(os.path.join(output_dir, '5_morphology_open_1.png'), img_morphology_1) #保存图片
cv2.imwrite(os.path.join(output_dir, '5_morphology_open_2.png'), img_morphology_2) #保存图片
print("任务二所有中间结果图已保存至 outputs/task2/ 目录下！")