import os
import shutil
import random

src_img = 'dataset/images/train'
src_lbl = 'dataset/labels/train'
dst_img = 'dataset/images/val'
dst_lbl = 'dataset/labels/val'

os.makedirs(dst_img, exist_ok=True)
os.makedirs(dst_lbl, exist_ok=True)

# 获取所有抽出来的图片
images = [f for f in os.listdir(src_img) if f.endswith('.jpg')]
random.shuffle(images) # 打乱顺序

# 抽 20% 给验证集
split_idx = int(len(images) * 0.8)
val_imgs = images[split_idx:]

for img in val_imgs:
    # 移动图片
    shutil.move(os.path.join(src_img, img), os.path.join(dst_img, img))
    # 移动对应的 txt 标签
    txt_name = img.replace('.jpg', '.txt')
    if os.path.exists(os.path.join(src_lbl, txt_name)):
        shutil.move(os.path.join(src_lbl, txt_name), os.path.join(dst_lbl, txt_name))

print(f"✅ 划分完成！把 {len(val_imgs)} 张图及标签分给了 val。")