import cv2
import numpy as np
import os
image_list = ['color1.jpg', 'color2.jpg', 'color3.jpg']
input_folder = '../data/images'
output_folder = '../outputs/task3'
os.makedirs(output_folder, exist_ok=True)
for img_name in image_list:
    print(f"\n================ 正在处理: {img_name} ================")
    img_path = os.path.join(input_folder, img_name)
    img = cv2.imread(img_path)

    if img is None:
        print(f"读取失败，请检查路径: {img_path}")
        continue
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    lower_orange = np.array([11, 100, 100])
    upper_orange = np.array([25, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask3 = cv2.inRange(hsv, lower_orange, upper_orange)
    mask = mask1 + mask2 + mask3
    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 去除背景上的小白点
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
    mask_save_path = os.path.join(output_folder, f'{os.path.splitext(img_name)[0]}_mask.png')
    cv2.imwrite(mask_save_path, mask_clean)
    print(f"已保存掩膜图: {mask_save_path}")
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = 500
    valid_contours = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > min_area:
            valid_contours += 1
            # 获取外接矩形 (x, y 是左上角坐标，w 是宽，h 是高)
            x, y, w, h = cv2.boundingRect(cnt)
            center_x = x + w // 2
            center_y = y + h // 2
            print(f"   -> 发现目标! 中心坐标: ({center_x}, {center_y}), 面积: {area:.1f} 像素")
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(img, (center_x, center_y), 5, (255, 0, 0), -1)

    if valid_contours == 0:
        print("未发现面积大于设定阈值的目标。")

    result_save_path = os.path.join(output_folder, f'{os.path.splitext(img_name)[0]}_result.jpg')
    cv2.imwrite(result_save_path, img)
    print(f" 已保存结果标注图: {result_save_path}")

print("\n任务三全部执行完毕！请去 outputs/task3 文件夹查看结果。")