import cv2
import numpy as np
import os

os.makedirs('W2/outputs/task1', exist_ok=True)

image_paths = [
    'W2/data/images/IMG1.jpg',
    'W2/data/images/IMG2.jpg',
    'W2/data/images/IMG3.jpg',
]

lower_white = np.array([0, 0, 120])
upper_white = np.array([180, 40, 255])

for path in image_paths:
    img = cv2.imread(path)
    if img is None:
        print(f"找不到图片: {path}")
        continue
    
    name = os.path.splitext(os.path.basename(path))[0]
    
    # 1. 转 HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 2. 生成掩膜
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # 3. 形态学清理
    kernel = np.ones((5,5), np.uint8) 
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    kernel = np.ones((7,7), np.uint8) 
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
    
    cv2.imwrite(f'W2/outputs/task1/{name}_mask.png', mask_clean)
    
    # 4. 找轮廓
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    result_img = img.copy()
    print(f"\n--- 处理图片: {name} ---")
    
    # 5. 筛选目标
    if len(contours) > 0:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        target_found = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1000:
                break
                
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h
            
            if 0.75 < aspect_ratio < 1.25:
                # 计算中心点
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # 画框和中心点
                    cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    cv2.circle(result_img, (cx, cy), 8, (0, 0, 255), -1) 
                    
                    # 写坐标文字
                    cv2.putText(result_img, f"u={cx}, v={cy}", (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    
                    print(f"检测成功！目标中心坐标: u={cx}, v={cy}, 轮廓面积: {area:.1f}")
                    target_found = True
                    break
        
        if not target_found:
            print("检测失败")
    else:
        print("检测失败")
    
    cv2.imwrite(f'W2/outputs/task1/{name}_result.png', result_img)

print("\n处理完成")