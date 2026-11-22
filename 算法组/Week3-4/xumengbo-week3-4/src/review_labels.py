"""
标签复核辅助（review_labels）
=============================
不替你改类别，只把需要人工回看的帧列出来：
  1. 空帧清单（有图无标签），按视频片段分组；
  2. 异常框：归一化宽/高过小(<0.02)或过大(>0.6)；
  3. 各类别框数与每帧框数统计。

用法：python review_labels.py train | val | test
报告输出到 D:\\W34_dataset\\markers\\review_<split>.txt
"""
import os
import sys
import glob
import re

split = sys.argv[1] if len(sys.argv) > 1 else "train"
ROOT = r"D:\W34_dataset\markers"
IMG_DIR = os.path.join(ROOT, "images", split)
LBL_DIR = os.path.join(ROOT, "labels", split)
REPORT = os.path.join(ROOT, f"review_{split}.txt")

CLASSES = ["LCross", "TCross", "XCross"]

images = sorted(glob.glob(os.path.join(IMG_DIR, "*.jpg")))
stems = {os.path.splitext(os.path.basename(p))[0]: p for p in images}

empty = []          # 空帧
abnormal = []       # 异常框
per_cls = [0, 0, 0]
frames_with_label = 0
multi_box = 0

for stem in sorted(stems):
    lp = os.path.join(LBL_DIR, stem + ".txt")
    if not os.path.exists(lp):
        empty.append(stem)
        continue
    frames_with_label += 1
    boxes = []
    with open(lp, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            cls = int(parts[0])
            cx, cy, bw, bh = [float(v) for v in parts[1:]]
            boxes.append((cls, cx, cy, bw, bh))
            if 0 <= cls <= 2:
                per_cls[cls] += 1
            if bw < 0.02 or bh < 0.02 or bw > 0.6 or bh > 0.6:
                abnormal.append((stem, CLASSES[cls], round(bw, 3), round(bh, 3)))
    if len(boxes) > 1:
        multi_box += 1

# 空帧按片段前缀分组（clip_02_f000000 -> clip_02）
def clip_of(stem):
    m = re.match(r"(clip_\d+)", stem)
    return m.group(1) if m else "unknown"

empty_by_clip = {}
for stem in empty:
    empty_by_clip.setdefault(clip_of(stem), []).append(stem)

lines = []
lines.append(f"===== {split} 标签复核报告 =====")
lines.append(f"图片总数: {len(stems)}  有标签帧: {frames_with_label}  空帧: {len(empty)}")
lines.append(f"多框帧: {multi_box}")
lines.append("各类框数: " + "  ".join(f"{CLASSES[c]}={per_cls[c]}" for c in range(3)))
lines.append("")
lines.append(f"--- 空帧清单（{len(empty)}，按片段分组，需回看是否漏标）---")
for clip, stems_list in sorted(empty_by_clip.items()):
    lines.append(f"[{clip}] {len(stems_list)} 帧")
    for s in stems_list:
        lines.append("    " + s)
lines.append("")
lines.append(f"--- 异常框（{len(abnormal)}，过小/过大，需回看）---")
for stem, cls, bw, bh in abnormal:
    lines.append(f"    {stem}  {cls}  w={bw} h={bh}")

report = "\n".join(lines)
with open(REPORT, "w", encoding="utf-8") as f:
    f.write(report)

print(report)
print("\n报告已保存:", REPORT)
