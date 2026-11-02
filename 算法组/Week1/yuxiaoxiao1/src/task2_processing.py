import cv2
import os
import numpy as np

out_dir="outputs/task2"
os.makedirs(out_dir,exist_ok=True)

image = cv2.imread("data/images/picture__two.png")

if image is not None:
    #灰度图
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    cv2.imwrite(os.path.join(out_dir,"1_gray.png"),gray)

    #高斯滤波
    gauss=cv2.GaussianBlur(image,(5,5),0)
    cv2.imwrite(os.path.join(out_dir,"2_gauss.png"),gauss)

    #阈值分割
    ret1, binary_low = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    ret2, binary_high = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    cv2.imwrite(os.path.join(out_dir,"3_binary_low.png"),binary_low)
    cv2.imwrite(os.path.join(out_dir, "3_binary_high.png"), binary_high)

    #canny边缘图
    canny_1 = cv2.Canny(gray, 20, 80)
    canny_2=cv2.Canny(gray,150,200)
    cv2.imwrite(os.path.join(out_dir,"4_canny_1.png"),canny_1)
    cv2.imwrite(os.path.join(out_dir, "4_canny_2.png"), canny_2)

    #形态学操作：开运算
    kernel = np.ones((10, 10), np.uint8)
    opening=cv2.morphologyEx(image,cv2.MORPH_OPEN,kernel)
    cv2.imwrite(os.path.join(out_dir,"5_erosion.png"),opening)

else:
    print("图片读取失败")
