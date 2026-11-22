# -*- coding: utf-8 -*-
"""
生成 3 张合成测试图（来源：程序生成，可复现）。
模拟任务书要求的"不同背景、光照、目标大小与颜色"：
  test1_dark_big.png   : 深色背景 + 橙色大矩形（暗背景 / 大目标）
  test2_bright_mid.png : 浅色背景 + 绿色大圆（亮背景 / 中等目标）
  test3_mid_small.png  : 中等亮度背景 + 红色小圆 + 蓝色小方块（多目标 / 小目标）

运行: python src/make_test_images.py
"""
import os
import cv2
import numpy as np

OUT = os.path.join("data", "images")
os.makedirs(OUT, exist_ok=True)


def make(rgb_bg, shapes, size=(640, 480)):
    """按背景色与形状列表生成一张 BGR 图像。
    shapes: [(kind, cx, cy, p1, p2, rgb)]，kind 支持 rect / circle / ellipse
    """
    img = np.full((size[1], size[0], 3), rgb_bg[::-1], dtype=np.uint8)  # RGB -> BGR
    for kind, cx, cy, p1, p2, rgb in shapes:
        color = rgb[::-1]
        if kind == "rect":
            cv2.rectangle(img, (cx - p1, cy - p2), (cx + p1, cy + p2), color, -1)
        elif kind == "circle":
            cv2.circle(img, (cx, cy), p1, color, -1)
        elif kind == "ellipse":
            cv2.ellipse(img, (cx, cy), (p1, p2), 0, 0, 360, color, -1)
    return img


# 图1：深蓝背景 + 橙色大矩形
img1 = make((20, 30, 80), [("rect", 320, 240, 140, 90, (255, 160, 40))])
# 图2：浅灰背景 + 绿色大圆
img2 = make((210, 210, 200), [("circle", 320, 240, 100, 0, (40, 180, 60))])
# 图3：中等亮度背景 + 红色小圆 + 蓝色小方块
img3 = make((120, 130, 110), [
    ("circle", 200, 240, 55, 0, (50, 60, 220)),
    ("rect", 440, 240, 60, 60, (220, 120, 30)),
])

cv2.imwrite(os.path.join(OUT, "test1_dark_big.png"), img1)
cv2.imwrite(os.path.join(OUT, "test2_bright_mid.png"), img2)
cv2.imwrite(os.path.join(OUT, "test3_mid_small.png"), img3)
print("Generated 3 test images under:", OUT)
