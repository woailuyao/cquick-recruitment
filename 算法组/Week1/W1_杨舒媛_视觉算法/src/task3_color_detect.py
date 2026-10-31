import cv2
import numpy as np
import os

os.makedirs('../outputs/task3', exist_ok=True)

image_paths = ['../data/image/2.jpg', '../data/image/3.jpg', '../data/image/4.jpg']

for i, path in enumerate(image_paths):
    print(f"\n=============================================")
    print(f"👉 正在处理第 {i + 1} 张测试图: {path}")

    img = cv2.imread(path)
    if img is None:
        print(f"❌ 找不到图片: {path}")
        continue

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 蓝色范围
    lower_blue = np.array([85, 40, 40])
    upper_blue = np.array([135, 255, 255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = img.copy()

    if contours:
        # 找最大轮廓
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        # 🌟 只保留下限，大于 1000 就画
        if area > 1000:
            x, y, w, h = cv2.boundingRect(c)
            center_x, center_y = x + w // 2, y + h // 2

            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.circle(result, (center_x, center_y), 8, (0, 0, 255), -1)

            print(f"   🎯 目标锁定！")
            print(f"   中心坐标 (x, y): ({center_x}, {center_y})")
            print(f"   轮廓面积: {area}")
        else:
            print(f"   ⚠️ 发现白色区域，但面积({area})太小，可能是噪点，已忽略。")

    # 保存
    cv2.imwrite(f'../outputs/task3/mask_{i + 2}.png', mask_clean)
    cv2.imwrite(f'../outputs/task3/result_{i + 2}.png', result)

    # 显示
    h_img, w_img = img.shape[:2]
    scale = 600 / w_img
    img_show = cv2.resize(result, (600, int(h_img * scale)))
    mask_show = cv2.resize(mask_clean, (600, int(h_img * scale)))

    print("   ⚠️ 请查看弹出的 Mask 和 Result 窗口，按任意键继续下一张...")
    cv2.imshow(f'[{i + 2}] Mask', mask_show)
    cv2.imshow(f'[{i + 2}] Result', img_show)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

print("\n✅ 全部测试完毕！所有结果已保存在 outputs/task3/ 文件夹中。")