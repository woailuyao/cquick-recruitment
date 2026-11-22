import os
root = r"D:\W34_dataset\ball"
lines = []
for split in ["train", "val", "test"]:
    img_dir = os.path.join(root, "images", split)
    lbl_dir = os.path.join(root, "labels", split)
    imgs = {os.path.splitext(f)[0] for f in os.listdir(img_dir) if f.lower().endswith((".jpg", ".jpeg", ".png"))}
    lbls = {os.path.splitext(f)[0] for f in os.listdir(lbl_dir) if f.lower().endswith(".txt") and f != "classes.txt"}
    miss_lbl = sorted(imgs - lbls)
    miss_img = sorted(lbls - imgs)
    lines.append(f"{split}: 图={len(imgs)} 标签={len(lbls)} 缺标签={len(miss_lbl)} 缺图={len(miss_img)}")
    if miss_lbl[:3]:
        lines.append("  缺标签例: " + str(miss_lbl[:3]))
    if miss_img[:3]:
        lines.append("  缺图例: " + str(miss_img[:3]))

# 抽一张图看尺寸
import cv2
sample = os.path.join(root, "images", "train", sorted(os.listdir(os.path.join(root, "images", "train")))[0])
im = cv2.imread(sample)
lines.append(f"样例尺寸: {im.shape[1]}x{im.shape[0]}  {os.path.basename(sample)}")

out = "\n".join(lines)
with open(r"D:\dataset_check.txt", "w", encoding="utf-8") as f:
    f.write(out)
print(out)
