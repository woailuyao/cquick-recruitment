import os
import shutil
import random

src_dir = 'dataset_ball/images/raw'
train_dir = 'dataset_ball/images/train'
val_dir = 'dataset_ball/images/val'
os.makedirs(train_dir, exist_ok=True)
os.makedirs(val_dir, exist_ok=True)
os.makedirs('dataset_ball/labels/train', exist_ok=True)
os.makedirs('dataset_ball/labels/val', exist_ok=True)

images = [f for f in os.listdir(src_dir) if f.endswith('.jpg')]
random.shuffle(images)

split_idx = int(len(images) * 0.8)
for img in images[:split_idx]:
    shutil.move(os.path.join(src_dir, img), os.path.join(train_dir, img))
for img in images[split_idx:]:
    shutil.move(os.path.join(src_dir, img), os.path.join(val_dir, img))
print("✅ 图片已按 8:2 划分到 train 和 val！")