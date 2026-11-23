# -*- coding: utf-8 -*-
"""按段生成拼图：每行一段，每段均匀取若干张。用于快速目检预标注质量。
用法：python make_contact_sheet.py [根目录] [输出路径] [每段取几张]
     默认根目录 raw、输出 _probe/contact_sheet.jpg、每段 6 张
"""
import cv2
import os
import sys
import numpy as np

ROOT = sys.argv[1] if len(sys.argv) > 1 else "raw"
OUT = sys.argv[2] if len(sys.argv) > 2 else "_probe/contact_sheet.jpg"
COLS = int(sys.argv[3]) if len(sys.argv) > 3 else 6
TW, TH = 256, 192


def imwrite_safe(p, i):
    ok, b = cv2.imencode(os.path.splitext(p)[1], i)
    if ok:
        b.tofile(p)


def put(img, text, org, scale=0.45, color=(0, 255, 255)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


segs = sorted(d for d in os.listdir(ROOT) if os.path.isdir(os.path.join(ROOT, d)))
rows = []
for seg in segs:
    files = sorted(f for f in os.listdir(os.path.join(ROOT, seg)) if f.lower().endswith(".jpg"))
    if not files:
        continue
    idxs = np.linspace(0, len(files) - 1, COLS).round().astype(int)
    row = []
    for j in idxs:
        img = cv2.imread(os.path.join(ROOT, seg, files[j]))
        if img is None:
            continue
        t = cv2.resize(img, (TW, TH), interpolation=cv2.INTER_AREA)
        put(t, f"{seg} #{j}/{len(files)}", (4, 16))
        row.append(t)
    while len(row) < COLS:
        row.append(np.zeros((TH, TW, 3), np.uint8))
    rows.append(np.hstack(row))

os.makedirs(os.path.dirname(OUT) or ".", exist_ok=True)
sheet = np.vstack(rows)
imwrite_safe(OUT, sheet)
print(f"{OUT}  {sheet.shape[1]}x{sheet.shape[0]}  （{len(rows)} 段）")
for seg in segs:
    n = len([f for f in os.listdir(os.path.join(ROOT, seg)) if f.lower().endswith(".jpg")])
    print(f"  {seg}: {n} 张")
