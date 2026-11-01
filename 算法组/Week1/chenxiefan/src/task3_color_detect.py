import cv2 as cv
import numpy as np
import os
import sys

#建立路径、导入图片
input_dir = os.path.join('..', 'data', 'images')
output_dir = os.path.join('..', 'outputs', 'task3')

#定义处理函数
def process(img_name, suffix):
    img_path = os.path.join(input_dir, img_name)
    img = cv.imread(img_path,1)
    if img is None:
        print("读取失败")
        sys.exit()
    else:
        print("读取成功,请按任意键继续")
    cv.imshow('MyWindow', img)
    cv.waitKey(0)
    cv.destroyAllWindows()

    #BGR转HSV
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower = np.array([35, 30, 20])
    upper = np.array([100, 255, 255])

    #生成掩膜
    print("生成掩膜")
    mask = cv.inRange(hsv, lower, upper)
    cv.imshow('Mask', mask)
    cv.waitKey(0)
    cv.destroyAllWindows()

    #执行开闭运算
    print("进行开闭运算")
    kernel = np.ones((10, 10), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

    #保存掩膜结果
    cv.imshow('Mask_processed', mask)
    mask_path = os.path.join(output_dir, f"mask{suffix}.png")
    result = cv.imwrite(mask_path,mask,[cv.IMWRITE_PNG_COMPRESSION, 3])
    cv.waitKey(0)
    cv.destroyAllWindows()

    #findContours 查找目标轮廓并过滤噪声区域
    print("查找轮廓")
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    blank_canvas = np.zeros_like(img)
    for cnt in contours:
        if cv.contourArea(cnt) > 10000:
            cv.drawContours(blank_canvas, [cnt], -1, (255, 255, 255), 2)
    cv.imshow('Contours on Blank', blank_canvas)
    cv.waitKey(0)
    cv.destroyAllWindows()

    #寻找中心点、画出外接矩形
    print("寻找中心点、画出外接矩形")
    result_img = img.copy()
    for cnt in contours:
        if cv.contourArea(cnt) > 10000:
            x, y, w, h = cv.boundingRect(cnt)
            center_x = x + w // 2
            center_y = y + h // 2
            cv.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.circle(result_img, (center_x, center_y), 5, (0, 0, 255), -1)
            print(f"中心坐标: ({center_x}, {center_y}), 轮廓面积: {cv.contourArea(cnt):.2f}")
    cv.imshow('Result', result_img)
    result_path = os.path.join(output_dir, f"Result{suffix}.png")
    result = cv.imwrite(result_path,result_img,[cv.IMWRITE_PNG_COMPRESSION, 3])
    cv.waitKey(0)
    cv.destroyAllWindows()

#调用函数
process('colored_image_1.jpg',1)
process('colored_image_2.jpg',2)
process('colored_image_3.jpg',3)
