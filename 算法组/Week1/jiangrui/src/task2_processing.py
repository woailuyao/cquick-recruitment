import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import pil_to_array

input_path = '算法组/Week1/jiangrui/data/images/img3.jpg'
output_dir = '算法组/Week1/jiangrui/outputs/task2'
os.makedirs(output_dir,exist_ok=True)
#读取图像并转换为灰度图
img = cv2.imread(input_path)
if img is None:
    print("Could not read the image")
    exit()
gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#高斯滤波对比
gauss3_img = cv2.GaussianBlur(gray_img,(3,3),0)
gauss33_img = cv2.GaussianBlur(gray_img,(33,33),0)

#固定阈值分割对比
_,thresh_127 = cv2.threshold(gray_img,127,255,cv2.THRESH_BINARY)
_,thresh_80 = cv2.threshold(gray_img,80,255,cv2.THRESH_BINARY)

#Canny边缘检测对比
canny_low = cv2.Canny(gauss3_img,50,150)
canny_high = cv2.Canny(gauss3_img,100,200)

#形态学开运算
kernel = np.ones((3,3),np.uint8)
morph_open = cv2.morphologyEx(thresh_80,cv2.MORPH_OPEN,kernel)

#保存结果
cv2.imwrite(os.path.join(output_dir,'gray.png'),gray_img)
cv2.imwrite(os.path.join(output_dir,'gaussian_3.png'),gauss3_img)
cv2.imwrite(os.path.join(output_dir,'gaussian_33.png'),gauss33_img)
cv2.imwrite(os.path.join(output_dir,'thresh_127.png'),thresh_127)
cv2.imwrite(os.path.join(output_dir,'thresh_80.png'),thresh_80)
cv2.imwrite(os.path.join(output_dir,'canny_50.png'),canny_low)
cv2.imwrite(os.path.join(output_dir,'canny_100.png'),canny_high)
cv2.imwrite(os.path.join(output_dir,'morph_open.png'),morph_open)

#可视化对比
imgs = [gray_img,gauss3_img,gauss33_img,thresh_127,thresh_80,canny_low,canny_high,morph_open]
titles =["Gray","Gauss3","Gauss33","Thresh_127","Thresh_80","Canny_low","Canny_high","morph_open"]

plt.figure(figsize=(15,6))
for i,(img,title) in enumerate(zip(imgs,titles)):
    plt.subplot(2,4,i+1)
    plt.imshow(img,cmap='gray')
    plt.title(title)
    plt.axis('off')
plt.show()
