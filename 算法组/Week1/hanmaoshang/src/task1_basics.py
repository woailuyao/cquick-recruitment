import cv2
import os

# 获取当前脚本(task1_basics.py)所在的绝对路径（即 src 文件夹）
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取项目根目录（即上一级目录 W1_...）
project_root = os.path.dirname(current_dir)

# 拼接出正确的图片路径
image_paths = [
    os.path.join(project_root, "data", "images", "image1.jpg"),
    os.path.join(project_root, "data", "images", "image2.jpg"),
    os.path.join(project_root, "data", "images", "image3.jpg")
]

# 拼接输出文件夹路径，并确保它存在
output_dir = os.path.join(project_root, "outputs", "task1")
os.makedirs(output_dir, exist_ok=True)

for i, path in enumerate(image_paths, 1):
    print(f"\n===== 处理第 {i} 张图片 =====")
    print(f"尝试读取路径: {path}")

    # 读取图片
    img = cv2.imread(path)  
    if img is None:
        print("❌ 读取失败！可能是图片名字不对，或者是中文路径导致OpenCV无法读取。")
        continue

    # 打印宽、高、通道数
    h, w, c = img.shape
    print(f"宽={w}, 高={h}, 通道数={c}")

    # 缩放
    resized = cv2.resize(img, None, fx=0.5, fy=0.5)
    cv2.imwrite(os.path.join(output_dir, f"{i}_resized.jpg"), resized)

    # 裁剪 (中心 200x200)
    center_x, center_y = w // 2, h // 2
    cropped = img[center_y-100:center_y+100, center_x-100:center_x+100]
    cv2.imwrite(os.path.join(output_dir, f"{i}_cropped.jpg"), cropped)

    # 灰度转换
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    cv2.imwrite(os.path.join(output_dir, f"{i}_gray.jpg"), gray)

    # 重新读取验证
    check = cv2.imread(os.path.join(output_dir, f"{i}_gray.jpg"))
    print(f"✅ 重新读取灰度图成功：shape={check.shape}")

print("\n任务一运行结束！快去 outputs/task1 看看吧！")