# -*- coding: utf-8 -*-
"""
生成可精确打印的标定棋盘格（A4 横向，300 DPI）
规格：10x7 个格子 = 9x6 个内角点，每个格子 25 mm，整板 250x175 mm
打印：选 A4、横向、缩放选「实际大小 / 100%」，不要选「适应页面」
验证：打印后用尺子量格子边长，应为 25 mm（图纸下方有一根 50 mm 校尺）
"""
from PIL import Image, ImageDraw, ImageFont

DPI = 300
MM = DPI / 25.4          # 每毫米多少像素 ≈ 11.811
A4_W, A4_H = int(297 * MM), int(210 * MM)   # 横向 A4: 3508 x 2480

COLS, ROWS = 10, 7       # 格子数（内角点 = 9x6）
SQ_MM = 20               # 格子边长 mm（标准 20 mm，便于用尺子校验）
SQ = int(SQ_MM * MM)     # 格子像素

img = Image.new("L", (A4_W, A4_H), 255)
d = ImageDraw.Draw(img)

board_w, board_h = COLS * SQ, ROWS * SQ
x0 = (A4_W - board_w) // 2
y0 = (A4_H - board_h) // 2 - int(8 * MM)    # 略上移，给下方说明留位置

# 画棋盘（(0,0) 为黑格，保证左上角是黑的）
for r in range(ROWS):
    for c in range(COLS):
        if (r + c) % 2 == 0:
            d.rectangle([x0 + c * SQ, y0 + r * SQ,
                         x0 + (c + 1) * SQ - 1, y0 + (r + 1) * SQ - 1], fill=0)

# 裁切线（虚线框，方便剪齐）
dash, gap = int(4 * MM), int(3 * MM)
for x in range(x0, x0 + board_w, dash + gap):
    d.line([x, y0 - int(2 * MM), min(x + dash, x0 + board_w), y0 - int(2 * MM)], fill=128, width=2)
    d.line([x, y0 + board_h + int(2 * MM), min(x + dash, x0 + board_w), y0 + board_h + int(2 * MM)], fill=128, width=2)
for y in range(y0, y0 + board_h, dash + gap):
    d.line([x0 - int(2 * MM), y, x0 - int(2 * MM), min(y + dash, y0 + board_h)], fill=128, width=2)
    d.line([x0 + board_w + int(2 * MM), y, x0 + board_w + int(2 * MM), min(y + dash, y0 + board_h)], fill=128, width=2)

# 底部说明文字
try:
    f_small = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", int(4.2 * MM))
    f_mid = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", int(5.2 * MM))
except Exception:
    f_small = f_mid = ImageFont.load_default()

ty = y0 + board_h + int(7 * MM)
d.text((x0, ty), f"标定棋盘格  10×7 格（内角点 9×6）  每格 {SQ_MM}.0 mm  整板 {COLS*SQ_MM}×{ROWS*SQ_MM} mm",
       fill=0, font=f_mid)
d.text((x0, ty + int(7 * MM)), "打印设置：A4 · 横向 · 缩放选「实际大小 / 100%」，不要选「适应页面」",
       fill=0, font=f_small)
d.text((x0, ty + int(13 * MM)), "校验：下方 50 mm 标尺用尺子量，必须正好 50 mm，否则重新打印",
       fill=0, font=f_small)

# 50mm 校尺（带刻度）
ry = ty + int(25 * MM)
d.line([x0, ry, x0 + int(50 * MM), ry], fill=0, width=int(0.6 * MM))
for i in range(51):
    h = int(3 * MM) if i % 10 == 0 else (int(2 * MM) if i % 5 == 0 else int(1.2 * MM))
    d.line([x0 + int(i * MM), ry, x0 + int(i * MM), ry - h], fill=0, width=2)
d.text((x0 + int(51 * MM), ry - int(2 * MM)), "50 mm 校尺", fill=0, font=f_small)
d.text((x0 + int(51 * MM), ry + int(2 * MM)), f"（OpenCV 9×6 内角点，SQUARE_MM={SQ_MM}.0）", fill=0, font=f_small)

img.save("chessboard_A4_print.png", dpi=(DPI, DPI))
img.save("chessboard_A4_print.pdf", "PDF", resolution=DPI)
print(f"已生成 chessboard_A4_print.png / .pdf  ({A4_W}x{A4_H} px @ {DPI} DPI)")
print("打印：A4 横向，缩放=实际大小/100%，量下方 50mm 校尺验证")
