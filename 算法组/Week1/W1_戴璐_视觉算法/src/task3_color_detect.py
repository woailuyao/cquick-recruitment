import cv2 as cv
import numpy as np
import os

#将要测试的三张图片加入列表
image_names = ['task3_color_detect_image1.jpg','task3_color_detect_image2.jpg','task3_color_detect_image3.jpg']
save_dir = 'outputs/task3'
os.makedirs(save_dir, exist_ok=True)

#for 循环，一次处理图片
for i, name in enumerate(image_names, start=1):
    print(f"正在处理第{i}张图:{name}...")
    
    image_path = f'data/images/{name}'
    img = cv.imread(image_path)

    if img is None:
        print("找不到图片，跳过")
        continue
    
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_orange = np.array([5,100,100])
    upper_orange = np.array([25,255,255])

    mask = cv.inRange(hsv, lower_orange, upper_orange)
    cv.imwrite(f'{save_dir}/img{i}_01_mask.png', mask)

    #形态学操作，改善掩膜
    kernel = np.ones((2, 2), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    #找轮廓
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    #按轮廓面积过滤噪声区， 画框， 和中心点
   
    if len(contours) > 0: # 只要找到了轮廓
        # 关键绝招：用 max() 找出面积最大的那个轮廓，其他的小噪点直接无视
        largest_contour = max(contours, key=cv.contourArea)
        
        # 计算这个最大轮廓的面积
        area = cv.contourArea(largest_contour)
        
        # 如果最大的轮廓面积也大于 500，说明是真目标
        if area > 500:
            # 获取最大轮廓的外接矩形
            x, y, w, h = cv.boundingRect(largest_contour)
            
            # 计算中心点
            center_x = x + w // 2
            center_y = y + h // 2
            
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
            
            text = f"Area:{int(area)}"
            cv.putText(img, text, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            print(f" 发现最大目标！中心坐标: ({center_x}, {center_y}), 面积: {int(area)}")
        else:
            print(" 找到轮廓了，但面积太小(<500)，可能是噪点。")

    cv.imwrite(f'{save_dir}/img{i}_02_result.png', img)
    print(f" 第{i}张图片处理完成，结果已存入 img{i}_02_result.png")







