"""HSV 取色助手：点击取色 + 自动统计目标/背景范围

用法：
  1. 点击模式下，在目标物体和背景上各点几下，观察 H/S/V
  2. 自动模式下，把目标颜色的大致范围填进 ROUGH，脚本统计真实分布

注意：S 很低（<40）时 H 是无意义的噪声，不要拿它做判断依据。
"""
from pathlib import Path
import cv2
import numpy as np

from io_utils import imread, imwrite

BASE = Path(__file__).resolve().parents[1]
IMG_DIR = BASE / "data" / "images"

IMAGE_NAME = "obj3.jpg"
MAX_SIDE = 800

# 只用于“自动统计”模式：一个很宽的粗略范围，用来圈出目标候选像素
ROUGH_LOWER = (95, 80, 40)
ROUGH_UPPER = (135, 255, 255)

img = imread(str(IMG_DIR / IMAGE_NAME))
if img is None:
    raise SystemExit(f"读取失败: {IMG_DIR / IMAGE_NAME}")

h, w = img.shape[:2]
scale = MAX_SIDE / max(h, w)
if scale < 1:
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

print(f"图像尺寸: {img.shape[1]}x{img.shape[0]}")


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        b, g, r = (int(v) for v in img[y, x])
        hh, ss, vv = (int(v) for v in hsv[y, x])
        flag = ""
        if ss < 40:
            flag = "   <- S 太低，H 无意义"
        print(f"({x},{y})  BGR=({b},{g},{r})  H={hh} S={ss} V={vv}{flag}")


# ---------- 自动统计 ----------
print("\n=== 自动统计：粗略范围内的像素分布 ===")
cand = cv2.inRange(hsv, np.array(ROUGH_LOWER), np.array(ROUGH_UPPER)) > 0
print(f"候选像素占比: {cand.mean() * 100:.2f}%")
if cand.any():
    for name, arr in [("H", H), ("S", S), ("V", V)]:
        p1, p5, p50, p95, p99 = np.percentile(arr[cand], [1, 5, 50, 95, 99]).astype(int)
        print(f"  {name}: p1={p1:3d}  p5={p5:3d}  中位={p50:3d}  p95={p95:3d}  p99={p99:3d}")

rest = ~cand
print("\n=== 非候选区域（背景）对照 ===")
for name, arr in [("H", H), ("S", S), ("V", V)]:
    p5, p50, p95 = np.percentile(arr[rest], [5, 50, 95]).astype(int)
    print(f"  {name}: p5={p5:3d}  中位={p50:3d}  p95={p95:3d}")
print(f"  背景中 S>80 的比例: {(S[rest] > 80).mean() * 100:.1f}%  (越低越好，说明背景不会误检)")

print("\n在窗口里点击目标亮部/暗部/边缘各几次，按 q 退出")

cv2.namedWindow("pick", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("pick", on_mouse)
while True:
    cv2.imshow("pick", img)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cv2.destroyAllWindows()

