# -*- coding: utf-8 -*-
"""把某个目录下的图拼成一张网格大图，方便一次看清。"""
import cv2
import os
import sys
import numpy as np

src = sys.argv[1]
out = sys.argv[2]
cols = int(sys.argv[3]) if len(sys.argv) > 3 else 6
TW = int(sys.argv[4]) if len(sys.argv) > 4 else 256


def imwrite_safe(p, i):
    ok, b = cv2.imencode(os.path.splitext(p)[1], i)
    if ok:
        b.tofile(p)


files = sorted(f for f in os.listdir(src) if f.lower().endswith(".jpg"))
if not files:
    print("目录里没有 jpg")
    sys.exit(1)
TH = int(TW * 0.75)
tiles = []
for f in files:
    img = cv2.imread(os.path.join(src, f))
    if img is None:
        continue
    t = cv2.resize(img, (TW, TH), interpolation=cv2.INTER_AREA)
    cv2.putText(t, f, (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(t, f, (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
    tiles.append(t)
while len(tiles) % cols:
    tiles.append(np.zeros((TH, TW, 3), np.uint8))
rows = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
sheet = np.vstack(rows)
imwrite_safe(out, sheet)
print(f"{out}  {sheet.shape[1]}x{sheet.shape[0]}  ({len(files)} 张)")
