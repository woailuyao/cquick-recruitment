import numpy as np
import matplotlib.pyplot as plt
import cv2
gray=cv2.imread("../images/image-sanyuanse.jpg", cv2.IMREAD_GRAYSCALE)
if gray is None:
    print("提取失败")
else:
    cv2.imshow("gray",gray)                   #灰度图

    blur1 = cv2.GaussianBlur(gray, (3, 3), 1.5)
    cv2.imshow("blur1", blur1)                  # 高斯滤波图(降噪，平滑图像）
    blur2 = cv2.GaussianBlur(gray, (5, 5), 0)
    cv2.imshow("blur2", blur2)

    canny1=(cv2.Canny(gray,100,200))                    #Canny边缘图（找到轮廓）
    cv2.imshow("canny1",canny1)
    canny2 = (cv2.Canny(gray, 150, 250))
    cv2.imshow("canny2", canny2)

    adaptive_1=cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,1)
                                                                           #默认前景白后景黑，INV交换，前景黑后景白； 分割的大小（奇数）
    cv2.imshow("adaptive_threshold1",adaptive_1) #阈值分割图（分出前后景）
    adaptive_2= cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 83, 1)
    cv2.imshow("adaptive_threshold2", adaptive_2)

    kernel1=np.ones((3,3),np.uint8)
    erosion1=cv2.erode(adaptive_1,kernel1)                #腐蚀(kernel覆盖处全白才才保持白色）
    dilation1=cv2.dilate(adaptive_1,kernel1)              #膨胀（kernel覆盖处有一点白就为白色）
    opening1=cv2.morphologyEx(adaptive_1,cv2.MORPH_OPEN,kernel1) #开运算（先腐蚀再膨胀）
    closing1=cv2.morphologyEx(adaptive_1,cv2.MORPH_CLOSE,kernel1) #闭运算（先膨胀再腐蚀）
    cv2.imshow("erosion1",erosion1)
    cv2.imshow("dilation1",dilation1)
    cv2.imshow("opening1",opening1)
    cv2.imshow("closing1",closing1)
    kernel2= np.ones((5,5), np.uint8)
    erosion2 = cv2.erode(adaptive_1, kernel2)  # 腐蚀
    dilation2 = cv2.dilate(adaptive_1, kernel2)  # 膨胀
    opening2 = cv2.morphologyEx(adaptive_1, cv2.MORPH_OPEN, kernel2)  # 开运算（先腐蚀再膨胀）
    closing2 = cv2.morphologyEx(adaptive_1, cv2.MORPH_CLOSE, kernel2)  # 闭运算（先膨胀再腐蚀）
    cv2.imshow("erosion2", erosion2)
    cv2.imshow("dilation2", dilation2)
    cv2.imshow("opening2", opening2)
    cv2.imshow("closing2", closing2)

    cv2.imwrite("../outputs/task2/gray.jpg", gray)
    cv2.imwrite("../outputs/task2/blur1.jpg", blur1)
    cv2.imwrite("../outputs/task2/blur2.jpg", blur2)
    cv2.imwrite("../outputs/task2/canny1.jpg", canny1)
    cv2.imwrite("../outputs/task2/canny2.jpg", canny2)
    cv2.imwrite("../outputs/task2/adaptive_threshold1.jpg", adaptive_1)
    cv2.imwrite("../outputs/task2/adaptive_threshold2.jpg", adaptive_2)
    cv2.imwrite("../outputs/task2/erosion1.jpg", erosion1)
    cv2.imwrite("../outputs/task2/dilation1.jpg", dilation1)
    cv2.imwrite("../outputs/task2/opening1.jpg", opening1)
    cv2.imwrite("../outputs/task2/closing1.jpg", closing1)
    cv2.imwrite("../outputs/task2/dilation2.jpg", dilation2)
    cv2.imwrite("../outputs/task2/erosion2.jpg", erosion2)
    cv2.imwrite("../outputs/task2/closing2.jpg", closing2)
    cv2.imwrite("../outputs/task2/opening2.jpg", opening2)
    cv2.waitKey()
    cv2.destroyAllWindows()
