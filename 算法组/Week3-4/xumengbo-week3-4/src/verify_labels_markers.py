"""
任务 2 标志标签 · 抽查与格式校验（verify_labels_markers）
=========================================================
两件事：
  1. 格式校验：类别号是否在 0~2、归一化坐标是否在 0~1、有无坏行；
     统计有标签帧数、总框数、各类框数、空帧数。
  2. 抽查画回：把标签框按类别颜色画回原图，输出到 D:\\W34_dataset\\verify_markers。

用法：python verify_labels_markers.py train | val | test
"""
import os
import sys
import glob
import cv2

split = sys.argv[1] if len(sys.argv) > 1 else "train"
ROOT = r"D:\W34_dataset\markers"
IMG_DIR = os.path.join(ROOT, "images", split)
LBL_DIR = os.path.join(ROOT, "labels", split)
OUT_DIR = r"D:\W34_dataset\verify_markers"
os.makedirs(OUT_DIR, exist_ok=True)

CLASSES = ["LCross", "TCross", "XCross"]
COLORS = [(0, 200, 0), (255, 128, 0), (0, 0, 255)]

# 收集图片与标签
images = {os.path.splitext(os.path.basename(p))[0]: p
          for p in glob.glob(os.path.join(IMG_DIR, "*.jpg"))}
label_files = [p for p in glob.glob(os.path.join(LBL_DIR, "*.txt"))
               if os.path.basename(p) != "classes.txt"]

total_boxes = 0
by_cls = [0, 0, 0]
bad_lines = []
labeled_stems = []

for lp in sorted(label_files):
    stem = os.path.splitext(os.path.basename(lp))[0]
    labeled_stems.append(stem)
    with open(lp, "r", encoding="utf-8") as f:
        for ln, line in enumerate(f, 1):
            parts = line.strip().split()
            if not parts:
                continue
            if len(parts) != 5:
                bad_lines.append(f"{stem}:{ln} 字段数={len(parts)}")
                continue
            cls = int(parts[0])
            vals = [float(v) for v in parts[1:]]
            if not (0 <= cls <= 2):
                bad_lines.append(f"{stem}:{ln} 类别号越界 cls={cls}")
            if any(v < 0 or v > 1 for v in vals):
                bad_lines.append(f"{stem}:{ln} 归一化坐标越界 {vals}")
            total_boxes += 1
            if 0 <= cls <= 2:
                by_cls[cls] += 1

# 有标签但找不到对应图片（异常）
orphan = [s for s in labeled_stems if s not in images]
empty_frames = len(images) - len(labeled_stems)

print(f"===== {split} 校验 =====")
print(f"图片总数: {len(images)}  有标签帧: {len(labeled_stems)}  空帧: {empty_frames}")
print(f"总框数: {total_boxes}")
for c, name in enumerate(CLASSES):
    print(f"  {name}({c}): {by_cls[c]} 框")
if orphan:
    print("⚠ 有标签无对应图片:", orphan)
if bad_lines:
    print("⚠ 格式错误行:")
    for b in bad_lines:
        print("   ", b)
else:
    print("格式校验: 通过")

# 抽查 6 张画回原图
step = max(1, len(labeled_stems) // 6)
picked = labeled_stems[::step][:6]
for stem in picked:
    img = cv2.imread(images[stem])
    if img is None:
        print("缺图:", stem)
        continue
    h, w = img.shape[:2]
    with open(os.path.join(LBL_DIR, stem + ".txt"), "r", encoding="utf-8") as fp:
        for line in fp:
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            cls = int(parts[0])
            cx, cy, bw, bh = [float(v) for v in parts[1:]]
            x1 = int((cx - bw / 2) * w)
            y1 = int((cy - bh / 2) * h)
            x2 = int((cx + bw / 2) * w)
            y2 = int((cy + bh / 2) * h)
            cv2.rectangle(img, (x1, y1), (x2, y2), COLORS[cls], 3)
            cv2.putText(img, CLASSES[cls], (x1, max(y1 - 8, 16)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLORS[cls], 2)
    cv2.imwrite(os.path.join(OUT_DIR, f"{split}_{stem}_chk.jpg"), img)
    print("抽查图:", f"{split}_{stem}_chk.jpg")

print("done，抽查图目录:", OUT_DIR)
