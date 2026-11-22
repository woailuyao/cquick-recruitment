"""
生成标定用棋盘格图片：8 列 x 6 行，设计边长 25 mm
打印后必须用尺子实测一格的真实边长，记录下来供标定求解使用。
"""
import os
import cv2
import numpy as np

COLS, ROWS = 8, 6          # 格子数；内角点 = (COLS-1, ROWS-1) = (7, 5)，与 snapshot_calib.py 一致
SQ_MM = 25.0               # 单个格子的设计边长，单位 mm（打印后要实测）
DPI = 300                  # 图片分辨率，越高格子边缘越锐利
BORDER_MM = 20.0           # 棋盘四周留白边，防止打印机裁掉边缘

px_per_mm = DPI / 25.4     # 每毫米对应多少像素
sq = round(SQ_MM * px_per_mm)      # 一个格子的像素边长
border = round(BORDER_MM * px_per_mm)

w = COLS * sq + 2 * border         # 整张图的像素宽
h = ROWS * sq + 2 * border         # 整张图的像素高

img = np.full((h, w), 255, np.uint8)   # 先铺白底

for r in range(ROWS):
    for c in range(COLS):
        if (r + c) % 2 == 0:          # 行列号之和为偶数的格子填黑
            x1 = border + c * sq
            y1 = border + r * sq
            cv2.rectangle(img, (x1, y1), (x1 + sq, y1 + sq), 0, -1)

# 外框：紧贴棋盘外缘画一圈黑线，把轮廓围清楚，防止白格与白纸背景混在一起
# 标定只识别格子之间的内角点，外框不参与求解，加它只是为了边界清晰
frame_w = round(2.0 * px_per_mm)
fx0, fy0 = border, border
fx1, fy1 = border + COLS * sq, border + ROWS * sq
cv2.rectangle(img, (fx0, fy0), (fx1, fy1), 0, frame_w)

OUT_DIR = r"D:\W34_dataset\calib"
os.makedirs(OUT_DIR, exist_ok=True)
out = os.path.join(OUT_DIR, "chessboard_8x6.png")
cv2.imwrite(out, img)

print("saved:", out)
print("格子数: %d x %d, 设计边长 %.1f mm, 格子像素边长 %d px" % (COLS, ROWS, SQ_MM, sq))
print("图片像素: %d x %d" % (w, h))
