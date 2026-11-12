import cv2
import os

# 准备要处理的图片列表
image_names = ["img1.jpg", "img2.jpg"]
input_dir = "data/images/"
output_dir = "outputs/task1/"

# 确保输出文件夹存在
os.makedirs(output_dir, exist_ok=True)

for name in image_names:
    image_path = os.path.join(input_dir, name)
    
    # 1. 读取图片并检查是否成功
    img = cv2.imread(image_path)
    if img is None:
        print(f"❌ 读取失败: {image_path}，请检查路径或文件名！")
        continue
    
    # 打印图像宽、高和通道数 (OpenCV 返回的是 高, 宽, 通道数)
    height, width, channels = img.shape
    print(f"✅ 成功读取 {name} | 尺寸: 宽{width} x 高{height}, 通道数: {channels}")

    # 获取不带后缀的文件名，用来做输出文件的名字
    file_prefix = name.split('.')[0]

    # 2. 分别完成缩放、指定区域裁剪和灰度转换
    # (1) 缩放：缩小为原来的一半
    resized_img = cv2.resize(img, (width // 2, height // 2))
    resize_path = os.path.join(output_dir, f"{file_prefix}_resized.jpg")
    cv2.imwrite(resize_path, resized_img)

    # (2) 指定区域裁剪：裁掉四周各100像素，保留中心区域
    cropped_img = img[100:height-100, 100:width-100]
    crop_path = os.path.join(output_dir, f"{file_prefix}_cropped.jpg")
    cv2.imwrite(crop_path, cropped_img)

    # (3) 灰度转换
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_path = os.path.join(output_dir, f"{file_prefix}_gray.jpg")
    cv2.imwrite(gray_path, gray_img)

    # 3. 检查保存文件是否能够重新读取
    # 随机抽查刚才保存的灰度图
    check_img = cv2.imread(gray_path)
    if check_img is not None:
        print(f"✅ {file_prefix}_gray.jpg 保存成功且重新读取通过！\n")
    else:
        print(f"❌ {file_prefix}_gray.jpg 重新读取失败！\n")

print("=== 任务一全部执行完毕，请去 outputs/task1/ 查看结果 ===")