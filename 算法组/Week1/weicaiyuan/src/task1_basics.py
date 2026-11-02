import cv2
import os

img_folder = 'data/images'
# 如果你是用 os.listdir 遍历的
for filename in os.listdir(img_folder):
    full_path = os.path.join(img_folder, filename)
    print(f"正在尝试读取: {full_path}") # 打印出来对比一下！
    img = cv2.imread(full_path)
    # ... 后续代码


# 1. 准备两张图片的路径（相对项目根目录）
image_paths = [
    "data/images/picture1.jpg",
    "data/images/picture2.jpg",
]

# 2. 输出目录，不存在会自动创建
out_dir = "outputs/task1"
os.makedirs(out_dir, exist_ok=True)

# 3. 依次处理每张图
for path in image_paths:
    img = cv2.imread(path)

    # 读取失败要明确提示，不能继续往下算
    if img is None:
        print(f"[失败] 读不到图片：{path}")
        continue

    # shape = (高, 宽, 通道)
    h, w, c = img.shape
    name = os.path.splitext(os.path.basename(path))[0]
    print(f"\n图片：{path}")
    print(f"宽={w}, 高={h}, 通道={c}")

    # 4. 缩放：注意 resize 参数是 (宽, 高)
    resized = cv2.resize(img, (w // 2, h // 2))
    resized_path = os.path.join(out_dir, f"{name}_resized.jpg")
    cv2.imwrite(resized_path, resized)

    # 5. 裁剪中间区域：img[y1:y2, x1:x2]
    y1, y2 = h // 4, 3 * h // 4
    x1, x2 = w // 4, 3 * w // 4
    cropped = img[y1:y2, x1:x2]
    cropped_path = os.path.join(out_dir, f"{name}_cropped.jpg")
    cv2.imwrite(cropped_path, cropped)

    # 6. 灰度转换
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_path = os.path.join(out_dir, f"{name}_gray.jpg")
    cv2.imwrite(gray_path, gray)

    # 7. 保存后重新读取，验证文件真的能打开
    for out_path in [resized_path, cropped_path, gray_path]:
        check = cv2.imread(out_path)
        if check is None:
            print(f"[回读失败] {out_path}")
        else:
            print(f"[回读成功] {out_path}, shape={check.shape}")

print("\n全部处理结束。请检查 outputs/task1/ 目录。")