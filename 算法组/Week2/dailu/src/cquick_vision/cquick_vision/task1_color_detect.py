import cv2
import numpy as np
import os


# 第2周 任务1：HSV颜色分割与目标中心点检测 (优化)


image_names = ['task1_01.jpg', 'task1_02.jpg', 'task1_03.jpg'] 

for i, name in enumerate(image_names, start=1):
    print(f"\n==============================")
    print(f"正在处理第 {i} 张图: {name} ...")
    
    image_path = f'{name}' 
    img = cv2.imread(image_path) 
    
    if img is None:
        print(f" 错误：找不到图片 {name}")
        continue 

    # 1. 转 HSV，提取橙色
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_color = np.array([5, 100, 100])
    upper_color = np.array([25, 255, 255])
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # 2. 形态学处理去噪
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # 保存掩膜图（中间结果）
    cv2.imwrite(f'01_mask_{i}.png', mask)

    # 3. 寻找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    max_area = 0

    # 4. 过滤干扰物
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # 面积下限设为 100，防止远距离小目标漏检
        if area > 100: 
            x, y, w_rect, h_rect = cv2.boundingRect(cnt)
            
            # 计算长宽比
            if w_rect > h_rect:
                aspect_ratio = float(w_rect) / h_rect
            else:
                aspect_ratio = float(h_rect) / w_rect
            
            # 长宽比放宽到 1.5，确保图1的笔不会被误伤
            if aspect_ratio > 1.5:
                if area > max_area:
                    max_area = area
                    best_contour = cnt

    # 5. 输出结果（绝对不画图）
    if best_contour is None:
        print(" 未检测到目标！(没有找到符合特征的目标)")
    else:
        x, y, w_rect, h_rect = cv2.boundingRect(best_contour)
        center_u = x + w_rect // 2
        center_v = y + h_rect // 2

        print(f" 成功检测到目标！")
        print(f"   中心坐标 (u, v): ({center_u}, {center_v})")
        print(f"   轮廓面积: {int(max_area)}")