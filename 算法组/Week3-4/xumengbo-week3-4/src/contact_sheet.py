"""生成某划分的缩略图总览网格，便于快速筛帧。用法: python contact_sheet.py val"""
import os, sys, glob, cv2, math
import numpy as np

split = sys.argv[1] if len(sys.argv) > 1 else "val"
IMG_DIR = rf"D:\W34_dataset\markers\images\{split}"
OUT = rf"D:\W34_dataset\markers\contact_{split}.jpg"

files = sorted(glob.glob(os.path.join(IMG_DIR, "*.jpg")))
cols = 4
tw, th = 320, 180
rows = math.ceil(len(files) / cols)
sheet = 255 * np.ones((rows * th, cols * tw, 3), dtype="uint8")

for i, f in enumerate(files):
    img = cv2.imread(f)
    if img is None:
        continue
    img = cv2.resize(img, (tw, th))
    r, c = divmod(i, cols)
    y, x = r * th, c * tw
    sheet[y:y+th, x:x+tw] = img
    name = os.path.splitext(os.path.basename(f))[0]
    cv2.rectangle(sheet, (x, y), (x+tw, y+22), (0, 0, 0), -1)
    cv2.putText(sheet, f"{i}:{name[-6:]}", (x+4, y+16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

cv2.imwrite(OUT, sheet)
print("frames:", len(files), "sheet:", OUT, "size:", sheet.shape)
