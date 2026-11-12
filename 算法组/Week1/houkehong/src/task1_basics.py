import cv2
import os

# 1. 定义项目内的相对路径
img_dir = "../data/images"
output_dir = "../outputs/task1"

# 确保输出目录存在
os.makedirs(output_dir, exist_ok=True)

# 定义要处理的图片名字（确保你的图片放在 data/images 下）
image_names = ["test1.jpg", "test2.jpg", "test3.jpg"]

for name in image_names:
    img_path = os.path.join(img_dir, name)

    # --- 要求1：读取并检查是否成功，打印宽、高和通道数 ---
    img = cv2.imread(img_path)
    if img is None:
        print(f"❌ 读取失败：找不到图片 {img_path}，请检查文件名或格式是否正确！")
        continue

    # shape 返回的是 (高度, 宽度, 通道数)
    h, w, c = img.shape
    print(f"✅ 成功读取 {name} | 宽度: {w}, 高度: {h}, 通道数: {c}")

    # --- 要求2：分别完成缩放、指定区域裁剪、灰度转换 ---
    # (1) 缩放：将图片宽高缩小一半
    resized_img = cv2.resize(img, (w // 2, h // 2))

    # (2) 裁剪：截取图像的中心区域 (高度取中间1/2，宽度取中间1/2)
    cropped_img = img[h // 4: h * 3 // 4, w // 4: w * 3 // 4]

    # (3) 灰度转换
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # --- 要求3：写入文件，检查能否重新读取，且不覆盖原图 ---
    base_name = os.path.splitext(name)[0]  # 获取不带后缀的文件名，如 "test1"

    resize_path = os.path.join(output_dir, f"{base_name}_resized.jpg")
    crop_path = os.path.join(output_dir, f"{base_name}_cropped.jpg")
    gray_path = os.path.join(output_dir, f"{base_name}_gray.jpg")

    # 保存图片
    cv2.imwrite(resize_path, resized_img)
    cv2.imwrite(crop_path, cropped_img)
    cv2.imwrite(gray_path, gray_img)
    print(f"💾 {name} 的处理结果已保存至 {output_dir}")

    # 重新读取验证
    check_img = cv2.imread(gray_path, cv2.IMREAD_GRAYSCALE)
    if check_img is not None:
        print(f"🔍 验证通过：{gray_path} 可被重新读取，尺寸为 {check_img.shape}")
    else:
        print(f"⚠️ 警告：{gray_path} 重新读取失败！")
    print("-" * 50)

print("🎉 任务一代码执行完毕！请去 outputs/task1 文件夹查看结果。")