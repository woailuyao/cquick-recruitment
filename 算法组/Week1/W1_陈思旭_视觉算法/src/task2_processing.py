import cv2
import numpy as np
import os

# 1. 设置路径
image_path = "data/images/img1.jpg"
output_dir = "outputs/task2/"
os.makedirs(output_dir, exist_ok=True)

# 2. 读取图片
img = cv2.imread(image_path)
if img is None:
    print("图片读取失败，请检查路径！")
else:
    print("开始处理图片...")

    # ----- 处理 1：灰度图 -----
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(os.path.join(output_dir, "1_gray.jpg"), gray)

    # ----- 处理 2：高斯滤波 -----
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    cv2.imwrite(os.path.join(output_dir, "2_gaussian_blur.jpg"), blurred)

    # ----- 处理 3：阈值分割 -----
    _, thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)
    cv2.imwrite(os.path.join(output_dir, "3_threshold_180.jpg"), thresh)

    # ----- 处理 4：Canny 边缘检测 -----
    # 50 和 150 是边缘检测的高低阈值
    edges = cv2.Canny(blurred, 100, 200)
    cv2.imwrite(os.path.join(output_dir, "4_canny_100_200_edges.jpg"), edges)

    # ----- 处理 5：形态学处理 -----
    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    cv2.imwrite(os.path.join(output_dir, "5_morphology_open.jpg"), opening)

    print("任务二处理完成！请去 outputs/task2/ 文件夹查看生成的5张图片。")