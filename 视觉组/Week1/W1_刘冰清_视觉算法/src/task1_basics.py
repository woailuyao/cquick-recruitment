#任务一：图像基础操作
#作者：刘冰清
#运行环境： Python 3.x, OpenCV
import cv2
import numpy as np
import os
input_img_path = "inputs/img1.jpg"
output_dir = "outputs/task1"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)
def process_image(image_path, suffix):
    print(f"开始处理图片: {image_path}")
    img = cv2.imread(image_path) #读取图片
    if img is None:
        print(f"[错误] 无法读取图片: {image_path}, 请检查路径是否正确。")
        return
    h, w, c = img.shape
    print(f"[任务1] 读取成功! 图片宽: {w}, 高: {h}, 通道数: {c}")
    new_w, new_h = w // 2, h // 2
    img_resized = cv2.resize(img, (new_w, new_h)) #缩放图片
    print(f"[操作1] 缩放完成 -> 新尺寸: {new_w}x{new_h}")
    crop_size = 100
    img_cropped = img
    if h > crop_size and w > crop_size:
        y1, y2 = (h - crop_size) // 2, (h + crop_size) // 2
        x1, x2 = (w - crop_size) // 2, (w + crop_size) // 2
        img_cropped = img[y1:y2, x1:x2] #裁剪图片
        print(f"[操作2] 裁剪完成 -> 区域: {crop_size}x{crop_size}")
    else:
        img_cropped = img
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #灰度转换
    print(f"[操作3] 灰度转换完成")
    save_path_resized = os.path.join(output_dir, f"{suffix}_resized.jpg")
    cv2.imwrite(save_path_resized, img_resized, [cv2.IMWRITE_JPEG_QUALITY, 95]) #保存图片
    check_resized = cv2.imread(save_path_resized) #读取验证
    if check_resized is not None:
        print(f"[第三步验证] 成功重新读取缩放图: {os.path.basename(save_path_resized)}")
    save_path_cropped = os.path.join(output_dir, f"{suffix}_cropped.jpg")
    cv2.imwrite(save_path_cropped, img_cropped, [cv2.IMWRITE_JPEG_QUALITY, 95])
    check_cropped = cv2.imread(save_path_cropped)
    if check_cropped is not None:
        print(f"[第三步验证] 成功重新读取裁剪图: {os.path.basename(save_path_cropped)}")
    save_path_gray = os.path.join(output_dir, f"{suffix}_gray.jpg")
    cv2.imwrite(save_path_gray, img_gray, [cv2.IMWRITE_JPEG_QUALITY, 95])
    check_gray = cv2.imread(save_path_gray)
    if check_gray is not None:
        print(f"[第三步验证] 成功重新读取灰度图: {os.path.basename(save_path_gray)}")
    print("-" * 30)
    return img_gray, img_resized, img_cropped
gray_img, resized_img, cropped_img = process_image(input_img_path, "result")

import cv2
import numpy as np
import os
input_img_path = "inputs/img2.png"
output_dir = "outputs/task1"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)
def process_image(image_path, suffix):
    print(f"开始处理图片: {image_path}")
    img = cv2.imread(image_path) #读取图片
    if img is None:
        print(f"[错误] 无法读取图片: {image_path}, 请检查路径是否正确。")
        return
    h, w, c = img.shape
    print(f"[任务1] 读取成功! 图片宽: {w}, 高: {h}, 通道数: {c}")
    new_w, new_h = w // 2, h // 2
    img_resized = cv2.resize(img, (new_w, new_h)) #缩放图片
    print(f"[操作1] 缩放完成 -> 新尺寸: {new_w}x{new_h}")
    crop_size = 100
    img_cropped = img
    if h > crop_size and w > crop_size:
        y1, y2 = (h - crop_size) // 2, (h + crop_size) // 2
        x1, x2 = (w - crop_size) // 2, (w + crop_size) // 2
        img_cropped = img[y1:y2, x1:x2] #裁剪图片
        print(f"[操作2] 裁剪完成 -> 区域: {crop_size}x{crop_size}")
    else:
        img_cropped = img
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #灰度转换
    print(f"[操作3] 灰度转换完成")
    save_path_resized = os.path.join(output_dir, f"{suffix}_resized.png")
    cv2.imwrite(save_path_resized, img_resized, [cv2.IMWRITE_JPEG_QUALITY, 95]) #保存图片
    check_resized = cv2.imread(save_path_resized) #读取验证
    if check_resized is not None:
        print(f"[第三步验证] 成功重新读取缩放图: {os.path.basename(save_path_resized)}")
    save_path_cropped = os.path.join(output_dir, f"{suffix}_cropped.png")
    cv2.imwrite(save_path_cropped, img_cropped, [cv2.IMWRITE_JPEG_QUALITY, 95])
    check_cropped = cv2.imread(save_path_cropped)
    if check_cropped is not None:
        print(f"[第三步验证] 成功重新读取裁剪图: {os.path.basename(save_path_cropped)}")
    save_path_gray = os.path.join(output_dir, f"{suffix}_gray.png")
    cv2.imwrite(save_path_gray, img_gray, [cv2.IMWRITE_JPEG_QUALITY, 95])
    check_gray = cv2.imread(save_path_gray)
    if check_gray is not None:
        print(f"[第三步验证] 成功重新读取灰度图: {os.path.basename(save_path_gray)}")
    print("-" * 30)
    return img_gray, img_resized, img_cropped
gray_img, resized_img, cropped_img = process_image(input_img_path, "result")