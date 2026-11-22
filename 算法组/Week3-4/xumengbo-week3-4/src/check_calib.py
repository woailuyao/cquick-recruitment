"""
标定棋盘检查：
1) 核对 chessboard_8x6.png 的像素尺寸、格子边长、能否检出 7x5 内角点；
2) 逐张检测 calib_XX.jpg 是否检出 7x5=35 个内角点，输出检出率与失败清单。
"""
import os
import cv2
import numpy as np

CALIB_DIR = r"D:\W34_dataset\calib"
BOARD_PNG = os.path.join(CALIB_DIR, "chessboard_8x6.png")
PATTERN = (7, 5)  # 内角点：8x6 格 -> 7x5

# ---------- 1. 棋盘图本身 ----------
print("=== 棋盘图 chessboard_8x6.png ===")
board = cv2.imread(BOARD_PNG, cv2.IMREAD_GRAYSCALE)
if board is None:
    print("  读不到棋盘图！")
else:
    h, w = board.shape
    print(f"  像素尺寸: {w} x {h}")
    # 检测内角点
    ok, corners = cv2.findChessboardCorners(board, PATTERN,
                                            cv2.CALIB_CB_ADAPTIVE_THRESH +
                                            cv2.CALIB_CB_NORMALIZE_IMAGE)
    print(f"  内角点检出: {ok}  数量={0 if corners is None else len(corners)} (应=35)")
    # 黑白取值抽查
    print(f"  灰度范围: min={board.min()} max={board.max()}")

# ---------- 2. 逐张实拍标定图 ----------
print("\n=== 实拍标定图检测结果 ===")
files = sorted(f for f in os.listdir(CALIB_DIR) if f.lower().startswith("calib_") and f.lower().endswith(".jpg"))
good, bad = [], []
for fn in files:
    p = os.path.join(CALIB_DIR, fn)
    img = cv2.imread(p)
    if img is None:
        bad.append((fn, "读不到"))
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, PATTERN,
                                            cv2.CALIB_CB_ADAPTIVE_THRESH +
                                            cv2.CALIB_CB_NORMALIZE_IMAGE)
    n = 0 if corners is None else len(corners)
    if ok and n == PATTERN[0] * PATTERN[1]:
        good.append(fn)
        print(f"  [OK]   {fn}  角点={n}  尺寸={img.shape[1]}x{img.shape[0]}")
    else:
        bad.append((fn, f"角点={n}"))
        print(f"  [失败] {fn}  角点={n}  尺寸={img.shape[1]}x{img.shape[0]}")

print(f"\n汇总: 共 {len(files)} 张, 检出 {len(good)} 张, 失败 {len(bad)} 张")
if bad:
    print("失败清单:", [b[0] for b in bad])
