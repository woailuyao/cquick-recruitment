import os
import shutil
import random

# 原始图片文件夹
src_dir = 'field_images'
# 目标数据集文件夹
base_dir = 'dataset'
os.makedirs(f'{base_dir}/images/train', exist_ok=True)
os.makedirs(f'{base_dir}/images/val', exist_ok=True)
os.makedirs(f'{base_dir}/labels/train', exist_ok=True)
os.makedirs(f'{base_dir}/labels/val', exist_ok=True)

# 获取所有图片
images = [f for f in os.listdir(src_dir) if f.endswith('.jpg')]
random.shuffle(images) # 打乱顺序

# 按 8:2 划分
split_idx = int(len(images) * 0.8)
train_imgs = images[:split_idx]
val_imgs = images[split_idx:]

# 复制图片
for img in train_imgs:
    shutil.copy(os.path.join(src_dir, img), os.path.join(f'{base_dir}/images/train', img))
for img in val_imgs:
    shutil.copy(os.path.join(src_dir, img), os.path.join(f'{base_dir}/images/val', img))

print(f"✅ 划分完成！训练集: {len(train_imgs)} 张，验证集: {len(val_imgs)} 张。")