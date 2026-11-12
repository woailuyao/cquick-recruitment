import cv2
import numpy as np
import os

img_path = "data/images/picture5.jpg"
out_dir = "outputs/task3"
os.makedirs(out_dir, exist_ok=True)

# 2. 读取图片并转换到 HSV
img = cv2.imread(img_path)
if img is None:
    raise FileNotFoundError(f"读不到图片：{img_path}")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# ==========================================
# 3. 橙色的 HSV 范围
# ==========================================
lower_color = np.array([6, 180, 150])
upper_color = np.array([19, 255, 255])

# 4. 生成掩膜（Mask）
mask = cv2.inRange(hsv, lower_color, upper_color)

# ==========================================
# 5. 形态学处理（开闭运算微调）
# ==========================================
# 闭运算：填补橘子表面的高光小黑洞
kernel_close = np.ones((17, 17), np.uint8)
mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

# 开运算：去噪的同时保护橘子边缘不被过度腐蚀
kernel_open = np.ones((1, 1), np.uint8)
mask_clean = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel_open)

# 6. 找轮廓
contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# ==========================================
# 7. 遍历每个轮廓，过滤并画框
# ==========================================
result_img = img.copy()

# 面积阈值：设定最小和最大面积
min_area = 5000
max_area = 300000

for cnt in contours:
    area = cv2.contourArea(cnt)

    # 过滤条件 1：面积必须在 min_area 和 max_area 之间
    if min_area < area < max_area:

        # 计算外接矩形 (x, y 是左上角坐标，w, h 是宽高)
        x, y, w, h = cv2.boundingRect(cnt)

        # 过滤条件 2：形状过滤（宽高比接近1，橘子是圆的）
        aspect_ratio = w / float(h)
        if aspect_ratio < 0.5 or aspect_ratio > 2.0:
            continue  # 如果是长条形，直接跳过

        # 在结果图上画绿色的矩形框
        cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 3)

        # 计算中心点
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # 在结果图上画红色的中心圆点
            cv2.circle(result_img, (cx, cy), 6, (0, 0, 255), -1)

            # 打印坐标和面积
            print(f"找到目标！中心坐标: ({cx}, {cy}), 轮廓面积: {area:.1f}")

# ==========================================
# 8. 保存结果（动态获取文件名，不覆盖）
# ==========================================
img_name = os.path.splitext(os.path.basename(img_path))[0]

mask_save_path = os.path.join(out_dir, f"{img_name}_mask.png")
result_save_path = os.path.join(out_dir, f"{img_name}_result.png")

cv2.imwrite(mask_save_path, mask_clean)
cv2.imwrite(result_save_path, result_img)

print(f"\n处理完成！已保存：{mask_save_path} 和 {result_save_path}")