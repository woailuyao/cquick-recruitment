import cv2 as cv
import numpy as np
easy=cv.imread("data/images/orange_easy.jpg")
dark=cv.imread("data/images/orange_dark.jpg")
noisy=cv.imread("data/images/orange_noisy.jpg")
if easy is not None and dark is not None and noisy is not None:
    hsv_easy=cv.cvtColor(easy,cv.COLOR_BGR2HSV)
    hsv_dark=cv.cvtColor(dark,cv.COLOR_BGR2HSV)
    hsv_noisy=cv.cvtColor(noisy,cv.COLOR_BGR2HSV)
    kernal=np.ones((3,3),np.uint8)
    
    #easy
    print('easy')
    lower_easy=np.array([5,130,110])
    upper_easy=np.array([22,255,255])
    mask_easy=cv.inRange(hsv_easy,lower_easy,upper_easy)#生成二值掩膜
    #形态学 先开后闭
    mask_easy=cv.morphologyEx(mask_easy,cv.MORPH_OPEN,kernal)
    mask_easy=cv.morphologyEx(mask_easy,cv.MORPH_CLOSE,kernal)
    cv.imwrite("outputs/task3/mask_easy.png",mask_easy)
    #寻找轮廓
    contours,_=cv.findContours(mask_easy,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    #开始画
    result_easy=easy.copy()
    for i in contours:
        area=cv.contourArea(i)
        #面积过滤
        if area>1000:
            #获取外接矩形边界 x，y是左上角坐标，w，h是宽高
            x,y,w,h=cv.boundingRect(i)
            #画绿色框（0，255，0） 线宽2
            cv.rectangle(result_easy,(x,y),(x+w,y+h),(0,255,0),2)
            #计算中心点
            center_x=x+w//2
            center_y=y+h//2
            #画红色实心中心点 (0,0,255) 半径5
            cv.circle(result_easy,(center_x,center_y),5,(0,0,255),-1)
            cv.imshow("result_easy",result_easy)
            cv.imwrite("outputs/task3/result_easy.png",result_easy)
            #输出
            print(f"找到目标中心({center_x},{center_y}),面积：{area}")

    #dark
    print('dark')
    lower_dark=np.array([5,130,110])
    upper_dark=np.array([22,255,255])
    mask_dark=cv.inRange(hsv_dark,lower_dark,upper_dark)#生成二值掩膜
        #形态学 先开后闭
    mask_dark=cv.morphologyEx(mask_dark,cv.MORPH_OPEN,kernal)
    mask_dark=cv.morphologyEx(mask_dark,cv.MORPH_CLOSE,kernal)
    cv.imwrite("outputs/task3/mask_dark.png",mask_dark)
        #寻找轮廓
    contours,_=cv.findContours(mask_dark,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
        #开始画
    result_dark=dark.copy()
    for j in contours:
        area1=cv.contourArea(j)
            #面积过滤
        if area1>1000:
                #获取外接矩形边界 x，y是左上角坐标，w，h是宽高
            x1,y1,w1,h1=cv.boundingRect(j)
                #画绿色框（0，255，0） 线宽2
            cv.rectangle(result_dark,(x1,y1),(x1+w1,y1+h1),(0,255,0),2)
                #计算中心点
            center_x1=x1+w1//2
            center_y1=y1+h1//2
                #画红色实心中心点 (0,0,255) 半径5
            cv.circle(result_dark,(center_x1,center_y1),5,(0,0,255),-1)
            cv.imshow("result_dark",result_dark)
            cv.imwrite("outputs/task3/result_dark.png",result_dark)
                #输出
            print(f"找到目标中心({center_x1},{center_y1}),面积：{area1}")

    #noisy
    print('noisy')
    lower_noisy=np.array([5,170,110])
    upper_noisy=np.array([22,255,255])
    mask_noisy=cv.inRange(hsv_noisy,lower_noisy,upper_noisy)#生成二值掩膜
        #形态学 先开后闭
    mask_noisy=cv.morphologyEx(mask_noisy,cv.MORPH_OPEN,kernal)
    mask_noisy=cv.morphologyEx(mask_noisy,cv.MORPH_CLOSE,kernal)
    cv.imwrite("outputs/task3/mask_noisy.png",mask_noisy)
        #寻找轮廓
    contours,_=cv.findContours(mask_noisy,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
        #开始画
    result_noisy=noisy.copy()
    for k in contours:
        area2=cv.contourArea(k)
            #面积过滤
        if area2>1000:
                #获取外接矩形边界 x，y是左上角坐标，w，h是宽高
            x2,y2,w2,h2=cv.boundingRect(k)
                #画绿色框（0，255，0） 线宽2
            cv.rectangle(result_noisy,(x2,y2),(x2+w2,y2+h2),(0,255,0),2)
                #计算中心点
            center_x2=x2+w2//2
            center_y2=y2+h2//2
                #画红色实心中心点 (0,0,255) 半径5
            cv.circle(result_noisy,(center_x2,center_y2),5,(0,0,255),-1)
            cv.imshow("result_noisy",result_noisy)
            cv.imwrite("outputs/task3/result_noisy.png",result_noisy)
                #输出
            print(f"找到目标中心({center_x2},{center_y2}),面积：{area2}")

    cv.waitKey(0)
else:
    print("图片读取错误！")