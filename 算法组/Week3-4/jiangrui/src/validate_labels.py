# -*- coding: utf-8 -*-
"""标注结果完整性校验：图片/标签配对、类别范围、坐标范围、退化框。

用法:
    python validate_labels.py                                  # 默认校验 marker_dataset
    python validate_labels.py D:/vision_data/ball_dataset ball  # 校验足球数据集
"""
import os
import sys
import glob
import collections

ROOT = sys.argv[1] if len(sys.argv) > 1 else r"D:/vision_data/marker_dataset"
NAMES = sys.argv[2].split(",") if len(sys.argv) > 2 else ["TCross", "LCross"]
print("校验目录:", ROOT, " 类别:", NAMES)

total_img = total_lab = 0
for split in ("train", "val", "test"):
    imgs = set(os.path.splitext(os.path.basename(p))[0]
               for p in glob.glob(os.path.join(ROOT, "images", split, "*.jpg")))
    labs = set(os.path.splitext(os.path.basename(p))[0]
               for p in glob.glob(os.path.join(ROOT, "labels", split, "*.txt")))
    miss_lab = sorted(imgs - labs)
    orphan = sorted(labs - imgs)

    cls = collections.Counter()
    bad_range = []
    degenerate = []
    empty = []
    nbox = 0
    for stem in sorted(labs):
        p = os.path.join(ROOT, "labels", split, stem + ".txt")
        boxes = []
        for line in open(p, encoding="utf-8"):
            q = line.split()
            if len(q) < 5:
                continue
            c = int(float(q[0]))
            cx, cy, bw, bh = map(float, q[1:5])
            cls[c] += 1
            nbox += 1
            boxes.append((c, cx, cy, bw, bh))
            if not all(0.0 <= v <= 1.0 for v in (cx, cy, bw, bh)):
                bad_range.append((stem, c))
            if bw * 1280 < 20 or bh * 720 < 20:
                degenerate.append((stem, c, round(bw * 1280), round(bh * 720)))
        if not boxes:
            empty.append(stem)

    total_img += len(imgs); total_lab += len(labs)
    print(f"[{split}] 图 {len(imgs)}  标签 {len(labs)}  框 {nbox}")
    print(f"    类别分布 {dict(sorted(cls.items()))}")
    print(f"    缺标签 {len(miss_lab)}  多余标签 {len(orphan)}  空标签 {len(empty)}")
    if miss_lab:
        print("      缺:", miss_lab[:10])
    if orphan:
        print("      多:", orphan[:10])
    if bad_range:
        print("    坐标越界:", bad_range[:10], "共", len(bad_range))
    if degenerate:
        print("    退化框:", degenerate[:10], "共", len(degenerate))
    if empty:
        print("    空标签文件:", empty[:10])

print("合计 图", total_img, "标签", total_lab)