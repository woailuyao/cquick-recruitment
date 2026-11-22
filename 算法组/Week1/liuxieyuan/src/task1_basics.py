import cv2
import os

# 定义图片路径（确保把 test1.jpg 和 test2.jpg 放在了 data/images 文件夹里）
img_paths = [
    "data/images/test1.jpg",
    "data/images/test2.jpg"
]

# 确保输出文件夹存在，如果没有就自动创建
os.makedirs("outputs/task1", exist_ok=True)

# 遍历两张图片进行处理
for i, path in enumerate(img_paths):
    print(f"\n========== 开始处理第 {i+1} 张图片: {path} ==========")
    
    # 1. 读取图片并检查是否成功
    img = cv2.imread(path)
    
    if img is None:
        print(f"❌ 读取失败！请检查文件是否存在且路径正确：{path}")
        continue
    
    # 打印图像宽、高和通道数 (h, w, c)
    height, width, channels = img.shape
    print(f"✅ 读取成功！图像尺寸: 宽={width}, 高={height}, 通道数={channels}")
    
    # 原图保存一份作为对比
    cv2.imwrite(f"outputs/task1/img{i+1}_original.jpg", img)
    
    # 2.1 缩放操作 (缩小到原来的一半)
    resized_img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    cv2.imwrite(f"outputs/task1/img{i+1}_resized.jpg", resized_img)
    print(f"✅ 缩放完成，新尺寸: {resized_img.shape}")
    
    # 2.2 指定区域裁剪 (截取中心位置 300x300 的区域)
    # 图像切片格式是 img[y_start:y_end, x_start:x_end]
    center_y, center_x = height // 2, width // 2
    crop_img = img[center_y-150 : center_y+150, center_x-150 : center_x+150]
    cv2.imwrite(f"outputs/task1/img{i+1}_cropped.jpg", crop_img)
    print(f"✅ 裁剪完成，裁剪区域尺寸: {crop_img.shape}")
    
    # 2.3 灰度转换
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(f"outputs/task1/img{i+1}_gray.jpg", gray_img)
    print(f"✅ 灰度转换完成")

    # 3. 检查保存的文件是否能重新读取
    check_path = f"outputs/task1/img{i+1}_gray.jpg"
    check_img = cv2.imread(check_path, cv2.IMREAD_GRAYSCALE)
    if check_img is not None:
        print(f"✅ 验证成功：保存的灰度图 {check_path} 可以被重新读取！")
    else:
        print(f"❌ 验证失败：无法读取保存的图片！")

print("\n🎉 任务一全部执行完毕！请去 outputs/task1 文件夹查看生成的图片。")