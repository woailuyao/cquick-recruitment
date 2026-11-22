import cv2
import numpy as np
import os

# 1. 设置路径：处理三张绿色苹果图
img_paths = [
    "data/images/test3.jpg",
    "data/images/test4.jpg",
    "data/images/test5.jpg"
]
os.makedirs("outputs/task3", exist_ok=True)

# 2. 循环处理每一张图
for i, img_path in enumerate(img_paths):
    print(f"\n========== 正在处理: {img_path} ==========")
    
    img = cv2.imread(img_path)
    if img is None:
        print(f"❌ 读取失败！请检查图片名字和后缀是否正确: {img_path}")
        continue

    # 转换到 HSV 颜色空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # ==========================================
    # 3. 设定【绿色】的 HSV 范围
    # ==========================================
    lower_green = np.array([35, 43, 46])
    upper_green = np.array([85, 255, 255])

    # 生成掩膜 (只有绿色区域变成白色)
    mask = cv2.inRange(hsv, lower_green, upper_green)
    cv2.imwrite(f"outputs/task3/mask_{i+1}.png", mask)

    # 4. 形态学开运算去噪（去掉周围细小的白色噪点）
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 5. 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # 找面积最大的轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # 过滤掉太小的噪点（面积大于 500 才认为是目标）
        if area > 500:
            # 画外接矩形（红色边框，粗细为3）
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 3)
            
            # 算中心点（绿色实心圆）
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.circle(img, (center_x, center_y), 5, (0, 255, 0), -1)
            
            print(f"✅ 找到目标！中心坐标: ({center_x}, {center_y})，面积: {area}")
        else:
            print("⚠️ 发现了绿色，但面积太小，可能是噪点。")
    else:
        print("⚠️ 未检测到绿色目标。")

    # 6. 保存结果图
    cv2.imwrite(f"outputs/task3/result_{i+1}.png", img)

print("\n🎉 任务三全部执行完毕！去 outputs/task3 查看 1、2、3 号结果图！")