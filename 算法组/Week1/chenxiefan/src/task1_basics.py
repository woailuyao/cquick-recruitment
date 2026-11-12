import cv2 as cv
import numpy as np
import os
import sys

#建立路径、导入图片
input_dir = os.path.join('..', 'data', 'images')
output_dir = os.path.join('..', 'outputs', 'task1')

#定义操作函数
def process_image(img_name, suffix):
    img_path = os.path.join(input_dir, img_name)
    img = cv.imread(img_path,1)

    #检查读取是否成功
    if img is None:
        print("读取失败")
        sys.exit()
    else:
        print("读取成功,请按任意键继续")

    #打印尺寸、调出图片
    print(img.shape)
    cv.imshow('My window',img)
    cv.waitKey(0)
    cv.destroyAllWindows()

    #进行缩放操作，检查操作后保存的图片
    resized = cv.resize(img, dsize=None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)
    roi = img[100:200,300:400].copy()
    gray_img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    resized_path = os.path.join(output_dir, f"out_resized{suffix}.png")
    result = cv.imwrite(resized_path,resized,[cv.IMWRITE_PNG_COMPRESSION, 3])
    cv.imshow('My window',cv.imread(resized_path,1))
    print("缩放操作成功,已读取操作后的图片,请按任意键继续")
    cv.waitKey(0)
    cv.destroyAllWindows()
    
    #进行裁剪操作，检查操作后保存的图片
    roi_path = os.path.join(output_dir, f"out_roi{suffix}.png")
    result = cv.imwrite(roi_path,roi,[cv.IMWRITE_PNG_COMPRESSION, 3])
    cv.imshow('My window',cv.imread(roi_path,1))
    print("裁剪操作成功,已读取操作后的图片,请按任意键继续")
    cv.waitKey(0)
    cv.destroyAllWindows()
    
    #进行灰度操作，检查操作后保存的图片
    gray_path = os.path.join(output_dir, f"out_gray{suffix}.png")
    result = cv.imwrite(gray_path,gray_img,[cv.IMWRITE_PNG_COMPRESSION, 3])
    cv.imshow('My window',cv.imread(gray_path,1))
    print("灰度操作成功,已读取操作后的图片,请按任意键继续")
    cv.waitKey(0)
    cv.destroyAllWindows()

#调用函数
process_image('SleepingCat.jpg', '1') 
process_image('Mouse.jpg', '2')
