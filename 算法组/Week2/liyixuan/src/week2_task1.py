import cv2
import numpy as np

# 1. 读取图片（注意你的图片路径，如果叫 2.jpeg 就改成 2.jpeg）
img = cv2.imread(r'D:\MyCode\data\images\1.jpeg')

# 2. 极其重要：检查图片有没有读进来
if img is None:
    print("❌ 错误：找不到图片，请检查路径！")
else:
    print("图片读取成功！开始检测...")
    
    # 3. 转成 HSV 颜色空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 4. 设定目标颜色（绿色）的 HSV 范围
    lower_green = np.array([35, 43, 46])
    upper_green = np.array([90, 255, 255])
    
    # 5. 生成掩膜
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # 6. 找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 7. 判断有没有找到东西
    if len(contours) == 0:
        print("❌ 未检测到目标！")
    else:
        # 8. 找面积最大的轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # 9. 过滤噪点
        if area < 500:
            print(f"检测到了物体，但面积({area})太小，判定为噪点。")
        else:
            # 10. 计算外接矩形，算出中心点 u, v
            x, y, w, h = cv2.boundingRect(largest_contour)
            u = x + w // 2
            v = y + h // 2
            
            # 终端输出坐标
            print(f"✅ 检测成功！目标面积: {area}")
            print(f"✅ 目标中心点坐标: u={u}, v={v}")
