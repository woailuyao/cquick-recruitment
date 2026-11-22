"""
抽查工具：把指定划分的标签框画回原图，输出核对图到 D:\\W34_dataset\\verify
用法：python verify_labels.py train | val | test
"""
import os
import sys
import cv2

split = sys.argv[1] if len(sys.argv) > 1 else "train"
IMG_DIR = rf"D:\W34_dataset\ball\images\{split}"
LBL_DIR = rf"D:\W34_dataset\ball\labels\{split}"
OUT_DIR = r"D:\W34_dataset\verify"
os.makedirs(OUT_DIR, exist_ok=True)

files = sorted(f for f in os.listdir(LBL_DIR) if f.endswith(".txt") and f != "classes.txt")
step = max(1, len(files) // 6)
picked = files[::step][:6]

for f in picked:
    stem = f[:-4]
    img = cv2.imread(os.path.join(IMG_DIR, stem + ".jpg"))
    if img is None:
        print("缺图:", stem)
        continue
    h, w = img.shape[:2]
    with open(os.path.join(LBL_DIR, f), "r", encoding="utf-8") as fp:
        for line in fp:
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            cx, cy, bw, bh = [float(v) for v in parts[1:]]
            x1 = int((cx - bw / 2) * w)
            y1 = int((cy - bh / 2) * h)
            x2 = int((cx + bw / 2) * w)
            y2 = int((cy + bh / 2) * h)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
    out_name = f"{split}_{stem}_chk.jpg"
    cv2.imwrite(os.path.join(OUT_DIR, out_name), img)
    print("checked:", out_name)
print("done")
