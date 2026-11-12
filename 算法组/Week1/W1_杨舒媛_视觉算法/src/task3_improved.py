import cv2
import numpy as np
import os

save_dir = '../outputs/task3_improved'
os.makedirs(save_dir, exist_ok=True)

path = '../data/image/4.jpg'
img = cv2.imread(path)

if img is not None:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 🌟 核心修改：调整 HSV 范围，只抓淡蓝色（天空区域）
    # 提高饱和度 S 的下限，避开灰白色的反光
    # 降低明度 V 的下限，避开深蓝色的马路和书
    lower_light_blue = np.array([85, 60, 150])
    upper_light_blue = np.array([105, 255, 255])

    mask = cv2.inRange(hsv, lower_light_blue, upper_light_blue)

    # 形态学清理噪点
    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = img.copy()

    if contours:
        # 找最大的淡蓝色轮廓（就是卡片的上面部分）
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > 1000:
            x, y, w, h = cv2.boundingRect(c)

            # 算出这半截卡片的中心点
            center_x = x + w // 2
            center_y = y + h // 2

            # 绘制框和红点
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.circle(result, (center_x, center_y), 8, (0, 0, 255), -1)

            print(f"Center: ({center_x}, {center_y}), Area: {area}")

    # 保存图片
    cv2.imwrite(f'{save_dir}/mask_4.png', mask_clean)
    cv2.imwrite(f'{save_dir}/result_4.png', result)

    # 显示
    h_img, w_img = img.shape[:2]
    scale = 600 / w_img
    img_show = cv2.resize(result, (600, int(h_img * scale)))
    mask_show = cv2.resize(mask_clean, (600, int(h_img * scale)))

    cv2.imshow('Mask (Light Blue Only)', mask_show)
    cv2.imshow('Result', img_show)
    cv2.waitKey(0)
    cv2.destroyAllWindows()