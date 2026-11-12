import cv2
import numpy as np
import os

image_path = "data/images/img4.jpg"  # 图片路径
output_dir = "outputs/task3/"
os.makedirs(output_dir, exist_ok=True)

# 读取图片
img = cv2.imread(image_path)
file_prefix = os.path.basename(image_path).split('.')[0]  # 获取文件名（不含后缀）

# 1. 缩小图片
scale_percent = 20 # 缩小到原来的 20%
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

# 2. 高斯滤波去噪
blurred = cv2.GaussianBlur(img, (5, 5), 0)

# 3. 转 HSV 并提取绿色
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])
mask = cv2.inRange(hsv, lower_green, upper_green)

# 4. 核心：加强形态学操作（把断裂的、零碎的地方全部连成一大块）

kernel = np.ones((15, 15), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 先开运算去掉外围小白点
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # 再闭运算把中间黑洞填满

cv2.imwrite(os.path.join(output_dir, f"mask_{file_prefix}.png"), mask)

# 5. 找轮廓
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

if len(contours) > 0:
    # 6. 直接找面积最大的那一个轮廓
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    
    # 打印面积信息
    print(f"最大目标面积: {area}")
    
    if area > 500:
        # 获取外接矩形并画框
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # 计算中心点并画红点
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
            
            text = f"({cx}, {cy}) Area:{int(area)}"
            cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print(f"找到目标！中心坐标: ({cx}, {cy})，面积: {int(area)}")

# 7. 保存结果
cv2.imwrite(os.path.join(output_dir, f"result_{file_prefix}.png"), img)
print("处理完成！请查看 outputs/task3/ 里的图片。")