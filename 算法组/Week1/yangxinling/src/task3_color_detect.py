import cv2
import numpy as np
import os

os.makedirs('W1/outputs/task3', exist_ok=True)

image_paths = [
    'W1/data/images/apple1.jpg',
    'W1/data/images/apple2.jpg',
    'W1/data/images/apple3.jpg'
]

# 针对红色的 HSV 范围（两个区间）
# 区间1：偏亮红/纯红 (H: 0~10)
lower_red1 = np.array([0, 100, 50])
upper_red1 = np.array([10, 255, 255])

# 区间2：偏紫红/暗红 (H: 170~180)
lower_red2 = np.array([170, 100, 50])
upper_red2 = np.array([180, 255, 255])

for path in image_paths:
    img = cv2.imread(path)
    if img is None:
        print(f"找不到图片: {path}")
        continue
    
    name = os.path.splitext(os.path.basename(path))[0]
    
    # 1. 转 HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 2. 生成两段掩膜并合并
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)  
    
    # 3. 形态学清理（开运算去除背景白点，闭运算填补苹果高光处的黑洞）
    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
    
    cv2.imwrite(f'W1/outputs/task3/{name}_mask.png', mask_clean)
    
    # 4. 找轮廓
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 获取图像的宽度和高度
    img_h, img_w = img.shape[:2]
    
    result_img = img.copy()
    print(f"\n--- 处理图片: {name} ---")
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # 过滤面积
        if area < 10000: 
            continue
            
        x, y, w, h = cv2.boundingRect(cnt)
        cx, cy = x + w // 2, y + h // 2

         # 过滤位置
        if cx < img_w * 0.25 or cx > img_w * 0.75 or cy < img_h * 0.25 or cy > img_h * 0.75:
            continue
        
        # 画框和中心点
        cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(result_img, (cx, cy), 5, (0, 255, 0), -1) # 中心点也画成绿色
        
        print(f"目标中心坐标: ({cx}, {cy}), 轮廓面积: {area}")
    
    cv2.imwrite(f'W1/outputs/task3/{name}_result.png', result_img)

print("\n处理完成")