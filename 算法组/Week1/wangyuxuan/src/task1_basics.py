import numpy as np
import matplotlib.pyplot as plt
import cv2
img=cv2.imread("../images/image2.jpg")                   #图片读取
if img is None:
    print('读取失败')
else:
    print(img.shape)                           #查看尺寸，通道数
    cv2.imshow("image",img)           #图片显示
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  #灰度转换
    cv2.imshow("gray",gray)

    crop=img[200:600,300:650]
    cv2.imshow("crop",crop)            #图片指定区域裁剪

    resized=cv2.resize(img,(300,300))     #指定宽高缩放或者按比例缩放resized=cv2.resize(img,None,fx=0.5,fy=0.5)
    cv2.imshow("outputs/task1/resized2.jpg",resized)
    cv2.imwrite("../outputs/task1/image2.jpg", img)
    cv2.imwrite("../outputs/task1/gray2.jpg", gray)
    cv2.imwrite("../outputs/task1/crop2.jpg", crop)
    cv2.imwrite("../outputs/task1/revised2.jpg", resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
