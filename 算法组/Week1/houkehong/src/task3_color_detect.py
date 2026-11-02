import cv2
import numpy as np
import os

img_dir = "../data/images"
output_dir = "../outputs/task3"
os.makedirs(output_dir, exist_ok=True)

# 你需要测试的图片列表（请改成你实际的图片名）
image_names = ["blue1.jpg", "blue2.jpg", "blue3.jpg"]

for name in image_names:
    img_path = os.path.join(img_dir, name)
    img = cv2.imread(img_path)
    if img is None:
        print(f"❌ 找不到图片: {img_path}")
        continue

    # 1. BGR 转换为 HSV 颜色空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 2. 设置 HSV 颜色范围并生成二值掩膜 (这里以蓝色为例)
    # 如果要找绿色，可以尝试: [35, 43, 46] 到 [77, 255, 255]
    # 如果要找橙色，可以尝试: [11, 43, 46] 到 [25, 255, 255]
    lower_color = np.array([100, 100, 50])  # 蓝色的下限
    upper_color = np.array([130, 255, 255])  # 蓝色的上限
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # 3. 形态学操作：开运算去除噪点，闭运算填补小孔
    # 定义一个 5x5 的卷积核
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 去噪
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 填补

    # 4. 查找轮廓
    # OpenCV 4.x/5.x 返回两个值：contours(轮廓列表), hierarchy(层级)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 5. 遍历轮廓，过滤面积并绘制
    # 面积阈值：小于这个面积的轮廓被认为是噪点
    min_area = 500

    result_img = img.copy()  # 在原图的副本上画图，保护原图

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > min_area:
            # 计算外接矩形
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 2)  # 画绿框

            # 计算中心点
            # 方法一：直接用外接矩形的几何中心
            # center_x = x + w // 2
            # center_y = y + h // 2

            # 方法二：使用图像矩 (Moments) 计算真实重心（更准确）
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
            else:
                center_x, center_y = x + w // 2, y + h // 2

            # 画中心点
            cv2.circle(result_img, (center_x, center_y), 5, (0, 0, 255), -1)  # 画红点

            # 打印坐标与面积
            print(f"✅ [{name}] 找到目标 | 中心坐标: ({center_x}, {center_y}) | 轮廓面积: {area:.1f}")

    # 6. 保存结果
    base_name = os.path.splitext(name)[0]
    mask_path = os.path.join(output_dir, f"{base_name}_mask.png")
    result_path = os.path.join(output_dir, f"{base_name}_result.png")

    cv2.imwrite(mask_path, mask)
    cv2.imwrite(result_path, result_img)
    print(f"💾 已保存掩膜图: {mask_path} 和 结果图: {result_path}\n")

print("🎉 任务三执行完毕！请去 outputs/task3 查看结果。")