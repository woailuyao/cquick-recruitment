#!/usr/bin/env python3
"""从 test(V5) 帧里抠出两类干扰球，作为 copy-paste 负样本素材。

只用检测框定位 + 颜色判据，不读取 test 的标注文件；抠出的 RGBA patch
仅是"外观素材"，贴到 train 图上时不生成任何标签（= 告诉模型这不是目标）。

两类干扰物：
  1) 黄绿 Molten 球：框内 HSV 黄绿连通域 + 凸包填充
  2) 红蓝花纹小球：检测框内取圆形区域（该球与蓝白目标球外观接近，
     仅花色调不同，故保留完整球面而不是只取红色像素）

用法（在训练工程根目录运行）：
    python src/extract_distractors.py \
        --baseline runs/ball_yolov8n/weights/best.pt \
        --green-aug runs/ball_yolov8n_aug/weights/best.pt
输出：green_patches/*.png、redblue_patches/*.png
"""
import argparse
import cv2
import numpy as np
from pathlib import Path
from ultralytics import YOLO

ap = argparse.ArgumentParser()
ap.add_argument("--test-dir", default="soccer_dataset/images/test")
ap.add_argument("--baseline", default="runs/ball_yolov8n/weights/best.pt")
ap.add_argument("--green-aug", default="runs/ball_yolov8n_aug/weights/best.pt")
a = ap.parse_args()

imgdir = Path(a.test_dir)


def green_component(crop):
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (20, 55, 60), (52, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None, 0.0
    idx = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    return (lab == idx).astype(np.uint8) * 255, stats[idx, cv2.CC_STAT_AREA] / (crop.shape[0] * crop.shape[1])


def red_ratio(crop):
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (0, 80, 70), (13, 255, 255))
    m2 = cv2.inRange(hsv, (165, 80, 70), (179, 255, 255))
    return float(((m1 > 0) | (m2 > 0)).mean())


def disk_mask(h, w, shrink=0.98):
    cy, cx = h / 2, w / 2
    Y, X = np.ogrid[:h, :w]
    rad = min(h, w) / 2 * shrink
    d = ((X - cx) ** 2 + (Y - cy) ** 2 <= rad ** 2).astype(np.uint8) * 255
    return cv2.GaussianBlur(d, (5, 5), 0)


# ---------- 1. 黄绿球 ----------
model = YOLO(a.baseline)
outg = Path("green_patches"); outg.mkdir(exist_ok=True)
raw = []
for img in sorted(imgdir.glob("*.jpg")):
    im = cv2.imread(str(img))
    for b in model(str(img), conf=0.25, verbose=False)[0].boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        pad = 4
        crop = im[max(0, y1 - pad):min(im.shape[0], y2 + pad),
                  max(0, x1 - pad):min(im.shape[1], x2 + pad)]
        if crop.size == 0:
            continue
        m, ratio = green_component(crop)
        if m is not None and ratio > 0.12:
            raw.append((crop, m, ratio))
raw.sort(key=lambda t: -t[2])
ng = 0
for i, (crop, mask, _) in enumerate(raw[:60]):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        continue
    solid = np.zeros_like(mask)
    cv2.drawContours(solid, [cv2.convexHull(max(cnts, key=cv2.contourArea))], -1, 255, -1)
    ys, xs = np.where(solid > 0)
    if len(xs) < 80:
        continue
    c = crop[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    mm = cv2.GaussianBlur(solid[ys.min():ys.max() + 1, xs.min():xs.max() + 1], (3, 3), 0)
    h, w = c.shape[:2]
    if h < 25 or w < 25 or abs(h - w) > max(h, w) * 0.55:
        continue
    rgba = cv2.cvtColor(c, cv2.COLOR_BGR2BGRA); rgba[:, :, 3] = mm
    cv2.imwrite(str(outg / f"green_{i:03d}.png"), rgba); ng += 1
print("黄绿球 patch:", ng)

# ---------- 2. 红蓝球 ----------
model = YOLO(a.green_aug)
outr = Path("redblue_patches"); outr.mkdir(exist_ok=True)
items = []
for img in sorted(imgdir.glob("*.jpg")):
    im = cv2.imread(str(img))
    for b in model(str(img), conf=0.15, verbose=False)[0].boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        crop = im[y1:y2, x1:x2]
        if crop.size == 0:
            continue
        rr = red_ratio(crop)
        if rr > 0.08:
            items.append((crop, rr))
items.sort(key=lambda t: -t[1])
uniq = items[::max(1, len(items) // 12)][:12]
for i, (crop, _) in enumerate(uniq):
    rgba = cv2.cvtColor(crop, cv2.COLOR_BGR2BGRA)
    rgba[:, :, 3] = disk_mask(*crop.shape[:2])
    cv2.imwrite(str(outr / f"redblue_{i:03d}.png"), rgba)
print("红蓝球 patch:", len(uniq))
