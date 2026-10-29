import cv2 as cv
img=cv.imread("data/images/car.jpg")
if img is None:
    print("图片读取错误！")
else:
    cv.imshow("img",img)
    
    img_gray=cv.cvtColor(img,cv.COLOR_BGR2GRAY) #输出灰度图
    cv.imshow("img_gray",img_gray)
    cv.imwrite("outputs/task2/car_black.jpg",img_gray)

    img_gaussian3=cv.GaussianBlur(img,(3,3),0) #高斯滤波（3，3） 平滑处理
    cv.imshow("img_gaussian3",img_gaussian3)
    cv.imwrite("outputs/task2/car_gaussian3.jpg",img_gaussian3)

    img_gaussian7=cv.GaussianBlur(img,(7,7),0) #高斯滤波（7，7） 平滑处理
    cv.imshow("img_gaussian7",img_gaussian7)
    cv.imwrite("outputs/task2/car_gaussian7.jpg",img_gaussian7)

    ret,img_threshl=cv.threshold(img_gray,80,255,cv.THRESH_BINARY) #低阈值
    cv.imshow("thresh_low",img_threshl)
    cv.imwrite("outputs/task2/car_thresh_low.jpg",img_threshl)

    ret,img_threshh=cv.threshold(img_gray,130,255,cv.THRESH_BINARY) #高阈值
    cv.imshow("thresh_high",img_threshh)
    cv.imwrite("outputs/task2/car_thresh_high.jpg",img_threshh)


    #img_thresha=cv.adaptiveThreshold(img_gray,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,9,0) #自适应阈值
    #cv.imshow("thresh_adaptive",img_thresha)
    #cv.imwrite("D:/CQU_CQuick/firstweek/outputs/task2/car_thresh_adaptive.jpg",img_thresha)
    

    img_edgel=cv.Canny(img_gaussian3,45,100) #边缘检测低阈值
    cv.imshow("edge_low",img_edgel)
    cv.imwrite("outputs/task2/car_edges_low.jpg",img_edgel)

    img_edgeh=cv.Canny(img_gaussian3,100,200) #边缘检测高阈值
    cv.imshow("edge_high",img_edgeh)
    cv.imwrite("outputs/task2/car_edges_high.jpg",img_edgeh)

    #形态学处理
    kernal=cv.getStructuringElement(cv.MORPH_RECT,(2,2)) #卷积核
    #开运算 先腐蚀后膨胀 去杂点杂线
    img_open=cv.morphologyEx(img_threshh,cv.MORPH_OPEN,kernal)
    cv.imshow("img_open",img_open)
    cv.imwrite("outputs/task2/car_open.jpg",img_open)
    #闭运算 先膨胀后腐蚀 
    img_close=cv.morphologyEx(img_threshh,cv.MORPH_CLOSE,kernal)
    cv.imshow("img_close",img_close)
    cv.imwrite("outputs/task2/car_close.jpg",img_close)

    cv.waitKey(0)