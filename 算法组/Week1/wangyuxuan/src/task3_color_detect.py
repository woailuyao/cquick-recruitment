import numpy as np
import matplotlib.pyplot as plt
import cv2
img=cv2.imread("../images/lan3.jpg")
if img is None:
    print('读取失败')
else:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 图象模型从BGR转变为HSV
    lower_blue = np.array([90, 80, 80])
    upper_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    cv2.imshow("mask.jpg", mask)

    kernel = np.ones((20, 20), np.uint8)
    maskclean= cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)     #闭运算清理白色区域内部黑噪点
    cv2.imshow("maskclean.jpg", maskclean)

    contours,_=cv2.findContours(maskclean,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) #查找目标轮廓，即勾出白色目标区域边缘,contours是列表，包含能识别出的所有轮廓
    area_=200
    img_copy=img.copy()                        #保留原图片
    for i in contours:                         #i是每一条被识别出的轮廓，但部分轮廓其实是噪点的轮廓所以需要筛选去除
        area = cv2.contourArea(i)
        if area > area_:
            x,y,w,h = cv2.boundingRect(i)      #算出每个轮廓的最小外接矩形的坐标点，（x,y)为左上角坐标，w,h为宽，高
            cv2.rectangle(img_copy,(x,y),(x+w,y+h),(0,255,0),2)    #画外接矩形并填充为绿色
            cx=int(x+w/2)
            cy=int(y+h/2)
            cv2.circle(img_copy,(cx,cy),5,(0,0,255),-1) #标出中心点填充为红色小圆
            print(f"中心坐标为({cx},{cy})")
            print(area)
            cv2.imshow("result.jpg",img_copy)
            cv2.imwrite("result.jpg",img_copy)
            cv2.imwrite("mask.jpg",maskclean)


    cv2.waitKey(0)
    cv2.destroyAllWindows()

