import cv2 as cv
img=cv.imread("data/images/cat1.jpg")
if img is None:
    print("图片读取错误！")
else:
    print(img.shape) #高、宽、通道数

    img_resize=cv.resize(img,(500,400)) #放缩(宽，高)
    cv.imwrite("outputs/task1/cat1_resize.jpg",img_resize)
    cv.imshow("img_resize",img_resize)
    
    img_crop=img[100:500,150:400] #裁剪[高度范围，宽度范围]
    cv.imwrite("outputs/task1/cat1_crop.jpg",img_crop)
    cv.imshow("img_crop",img_crop)
    
    img_gray=cv.cvtColor(img,cv.COLOR_BGR2GRAY) #灰度转换
    cv.imwrite("outputs/task1/cat1_black.jpg",img_gray) #保存灰度图
    cv.imshow("img_black",img_gray)

    cv.waitKey(0)
    print(type(img))

im1=cv.imread("outputs/task1/cat1_resize.jpg")
im2=cv.imread("outputs/task1/cat1_crop.jpg")
im3=cv.imread("outputs/task1/cat1_black.jpg")
if im1 is not None and im2 is not None and im3 is not None:
    print("PERFECT!")
