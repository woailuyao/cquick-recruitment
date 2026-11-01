#任务三：颜色目标提取与轮廓定位
#作者：刘冰清
#运行环境： Python 3.x, OpenCV
import cv2
import os
import numpy as np
output_dir = 'outputs/task3'
os.makedirs(output_dir, exist_ok=True)
def process_image(img_path, lower_hsv, upper_hsv, min_area=500):
    img = cv2.imread(img_path) #读取图片
    if img is None:
        print(f"找不到图片 '{img_path}', 请检查文件名和路径! ")
        return
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #颜色转换
    if lower_hsv.max() <= 1.0:
        img_rgb = img.copy()
        mask = cv2.inRange(img_rgb, (lower_hsv * 255).astype(np.uint8), (upper_hsv * 255).astype(np.uint8)) #颜色筛选
    else:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #颜色转换
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv) #颜色筛选
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)) #定义”核“
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #找轮廓
    result = img.copy()
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(contour)
            M = cv2.moments (contour)
            if M["m00"] != 0:
                cx = int(M["m10"] // M["m00"])
                cy = int(M["m01"] // M["m00"])
            else:
                cx, cy = x + w//2, y + h//2
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 15) #画框
            cv2.circle(result, (cx, cy), 4, (0, 0, 255), -1)
            print(f"目标区域 - 中心点: ({cx}, {cy}), 面积: {area:.1f}")
    base_name = os.path.splitext(os.path.basename(img_path))[0]
    mask_path = os.path.join(output_dir, f'mask_{base_name}.png')
    result_path = os.path.join(output_dir, f'result_{base_name}.png')
    cv2.imwrite(mask_path, mask) #保存图片
    cv2.imwrite(result_path, result)
    print(f"已保存: {mask_path}")
    print(f"已保存: {result_path}\n")
tasks = [
    {"path": "inputs/img1.jpg", "lower": np.array([20, 100, 100]), "upper": np.array([35, 255, 255])},
    {"path": "inputs/img2.png", "lower": np.array([35, 40, 40]), "upper": np.array([85, 255, 255])},
    {"path": "inputs/img3.png", "lower": np.array([35, 40, 40]), "upper": np.array([85, 255, 255])}]
print(f"开始处理所有图片\n")
for task in tasks:
    print(f"正在提取图片 '{task['path']}' 的颜色区域")
    process_image(task["path"], task["lower"], task["upper"], min_area=300)
    print("_" * 30)