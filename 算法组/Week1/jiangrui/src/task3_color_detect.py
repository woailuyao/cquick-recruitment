import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
# 设置路径
out_dir = '算法组/Week1/jiangrui/outputs/task3'
os.makedirs(out_dir, exist_ok=True)

# 准备3张图片
image_names = ['img2.jpg', 'img4.jpg', 'img5.jpg']

# 设置红色的HSV范围（红色横跨0和180两端，所以分两段）
lower_red1 = np.array([0, 80, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 80, 50])
upper_red2 = np.array([179, 255, 255])

# 开始循环处理每一张图
for i, name in enumerate(image_names):
    input_path = os.path.join('算法组/Week1/jiangrui/data/images', name)
    img = cv2.imread(input_path)

    if img is None:
        print(f"找不到图片 {input_path}，请检查！")
        continue

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 提取红色并合并
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 形态学处理：开运算去掉小白点噪声
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result_img = img.copy()

    if len(contours) > 0:
        # 取面积最大的轮廓
        biggest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest_contour)

        # 面积太小就认为不是目标
        if area > 500:
            x, y, w, h = cv2.boundingRect(biggest_contour)
            cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 画中心点
            cx = x + w // 2
            cy = y + h // 2
            cv2.circle(result_img, (cx, cy), 5, (255, 0, 0), -1)

            print(f"  {name} 找到红色目标，面积: {area}, 中心: ({cx}, {cy})")
        else:
            print(f"  {name} 面积太小({area})，跳过")
    else:
        print(f"  {name} 没有找到红色目标")

    # 保存结果
    cv2.imwrite(os.path.join(out_dir, f'mask_{i + 1}.png'), mask)
    cv2.imwrite(os.path.join(out_dir, f'result_{i + 1}.png'), result_img)

print("\n全部处理完毕！请去 outputs/task3 文件夹查看结果。")

# 显示结果
plt.figure(figsize=(15, 10))
for i in range(3):
    res_img = cv2.imread(os.path.join(out_dir, f'result_{i+1}.png'))
    if res_img is not None:
        res_img_rgb = cv2.cvtColor(res_img, cv2.COLOR_BGR2RGB)
        plt.subplot(1, 3, i+1)
        plt.title(f"Result {i+1} ({image_names[i]})")
        plt.imshow(res_img_rgb)
        plt.axis('off')

plt.tight_layout()
plt.show()
