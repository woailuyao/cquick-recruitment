import os
import json
import shutil
import random

base_dir = r"D:\Marker_Dataset"
random.seed(42)

# 创建 YOLO 标准目录
for split in ['train', 'val', 'test']:
    os.makedirs(os.path.join(base_dir, 'images', split), exist_ok=True)
    os.makedirs(os.path.join(base_dir, 'labels', split), exist_ok=True)

# 精确搜索 all_markers 文件夹下的所有 json
json_files = []
for root, dirs, files in os.walk(os.path.join(base_dir, "all_markers")):
    for file in files:
        if file.endswith(".json"):
            json_files.append(os.path.join(root, file))

print(f"共找到 {len(json_files)} 个标注文件，开始转换...")



class_mapping = {
    "LCross": 0,
    "TCross": 1,
    "XCross": 2,
    "PenaltyPoint": 3
}

count = 0
for json_path in json_files:
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    img_name = data.get("imagePath")
    if not img_name:
        img_name = os.path.basename(json_path).replace('.json', '.jpg')

    img_path = os.path.join(os.path.dirname(json_path), img_name)
    if not os.path.exists(img_path):
        continue

    w = data.get("imageWidth")
    h = data.get("imageHeight")

    yolo_lines = []
    for shape in data.get("shapes", []):
        label = shape.get("label")
        if label not in class_mapping:
            continue

        class_id = class_mapping[label]
        points = shape.get("points")

        if len(points) == 4:
            x_coords = [p[0] for p in points]
            y_coords = [p[1] for p in points]
            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)

            bw = (x_max - x_min) / w
            bh = (y_max - y_min) / h
            cx = (x_min + x_max) / 2.0 / w
            cy = (y_min + y_max) / 2.0 / h

            yolo_lines.append(f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")

    if not yolo_lines:
        continue

    rand_val = random.random()
    split = 'train' if rand_val < 0.8 else ('val' if rand_val < 0.9 else 'test')

    new_img_path = os.path.join(base_dir, 'images', split, img_name)
    new_txt_path = os.path.join(base_dir, 'labels', split, img_name.replace('.jpg', '.txt'))

    shutil.copy(img_path, new_img_path)
    with open(new_txt_path, 'w') as f:
        f.writelines(yolo_lines)
    count += 1

print(f"转换完毕！成功处理了 {count} 张带有标注的图片。")