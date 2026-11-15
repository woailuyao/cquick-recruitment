import numpy as np
import matplotlib.pyplot as plt
import cv2
img=cv2.imread("../image/huang1.png")
if img is None:
    print('读取失败')
else:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 图象模型从BGR转变为HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    cv2.namedWindow('mask.png', cv2.WINDOW_NORMAL)    #缩放到正常大小
    cv2.imshow("mask.png", mask)       #二值掩膜

    kernel = np.ones((100, 100), np.uint8)
    maskclean= cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)     #闭运算清理白色区域内部黑噪点
    cv2.namedWindow('maskclean.png', cv2.WINDOW_NORMAL)
    cv2.imshow("maskclean.png", maskclean)

    contours,_=cv2.findContours(maskclean,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) #查找目标轮廓，即勾出白色目标区域边缘,contours是列表，包含能识别出的所有轮廓
    area_=16000
    img_copy=img.copy()                        #保留原图片
    lst=[]
    dict_={}
    for i in contours:                         #i是每一条被识别出的轮廓，但部分轮廓其实是噪点的轮廓所以需要筛选去除
        area = cv2.contourArea(i)
        if area > area_:
            x,y,w,h = cv2.boundingRect(i)      #算出每个轮廓的最小外接矩形的坐标点，（x,y)为左上角坐标，w,h为宽，高
            cv2.rectangle(img_copy,(x,y),(x+w,y+h),(0,255,0),2)    #画外接矩形并填充为绿色
            cx=int(x+w/2)
            cy=int(y+h/2)
            cv2.circle(img_copy,(cx,cy),5,(0,0,255),-1) #标出中心点填充为红色小圆
            lst.append(area)
            dict_[area]={"x":cx,"y":cy}

    if not lst:
        print("未检测到目标")
    else:
        lst.sort(reverse=True)
        print(f"中心坐标为({dict_[lst[0]]["x"]},{dict_[lst[0]]["y"]})")
        print(lst[0])
        text = f"({cx},{cy})"
        cv2.putText(img_copy, text, (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # 输出中心点坐标文字
        cv2.namedWindow('result.png', cv2.WINDOW_NORMAL)
        cv2.imshow("result.png", img_copy)
        '''cv2.imwrite("result.jpg",img_copy)
        cv2.imwrite("mask.jpg",maskclean)'''

        cv2.waitKey(0)
        cv2.destroyAllWindows()

