# -*- coding: utf-8 -*-
"""生成可打印的标定棋盘格（A4 横向，1:1 尺寸）。

为什么要自己生成：网上模板的「9x6」常指方格数、且打印缩放不一定 100%，
两者都会让标定失败或尺度出错。这里把尺寸写死在像素与毫米上，并留出核对区。

规格：
    方格 25.0 mm，10 x 7 个方格 -> 内角点 9 x 6
    A4 横向 297 x 210 mm，300 dpi（3508 x 2480 px）
    棋盘区域 250 x 175 mm，居中，四周留白便于检测

打印要求：
    **必须按 100% / 实际大小打印**，不要「适合页面」。
    打印后用量尺复核：任意 10 个方格的跨度应为 250 mm（差 1 mm 内可接受）。
    若偏差大，用实测值算 square_mm 传给 calibrate_intrinsics.py：
        square_mm = 实测跨度_mm / 10

用法：
    python tools/make_chessboard.py
    python tools/make_chessboard.py --squares-x 10 --squares-y 7 --mm 25
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import OUTPUT_DIR, rel          # noqa: E402


def make_board(squares_x: int, squares_y: int, mm_per_sq: float,
               dpi: int = 300, paper: str = "A4-landscape") -> np.ndarray:
    """生成 1:1 的棋盘格图。squares_x/y 为方格数。"""
    paper_mm = {"A4-landscape": (297.0, 210.0), "A4-portrait": (210.0, 297.0)}[paper]
    px_per_mm = dpi / 25.4
    W = int(round(paper_mm[0] * px_per_mm))
    H = int(round(paper_mm[1] * px_per_mm))
    img = np.full((H, W), 255, np.uint8)

    sq_px = mm_per_sq * px_per_mm
    bw = int(round(squares_x * sq_px))
    bh = int(round(squares_y * sq_px))
    x0 = (W - bw) // 2
    y0 = (H - bh) // 2 - int(round(6 * px_per_mm))     # 略上移，底部留文字区

    for j in range(squares_y):
        for i in range(squares_x):
            if (i + j) % 2 == 0:
                x1 = x0 + int(round(i * sq_px))
                y1 = y0 + int(round(j * sq_px))
                x2 = x0 + int(round((i + 1) * sq_px))
                y2 = y0 + int(round((j + 1) * sq_px))
                cv2.rectangle(img, (x1, y1), (x2, y2), 0, -1)

    # 细边框，便于确认没有被裁切
    cv2.rectangle(img, (x0 - 2, y0 - 2), (x0 + bw + 1, y0 + bh + 1), 0, 2)

    text_y = y0 + bh + int(round(14 * px_per_mm))
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = px_per_mm * 0.32
    cv2.putText(img, f"squares {squares_x} x {squares_y}   "
                     f"inner corners {squares_x - 1} x {squares_y - 1}   "
                     f"square {mm_per_sq:.1f} mm",
                (int(0.06 * W), text_y), font, scale, 0, 2)
    cv2.putText(img, f"PRINT AT 100% (actual size), {dpi} dpi, {paper}. "
                     f"Check: 10 squares span = {10 * mm_per_sq:.0f} mm",
                (int(0.06 * W), text_y + int(round(9 * px_per_mm))), font, scale * 0.85, 0, 2)
    return img


def main() -> int:
    ap = argparse.ArgumentParser(description="生成可打印棋盘格")
    ap.add_argument("--squares-x", type=int, default=10, help="方格列数")
    ap.add_argument("--squares-y", type=int, default=7, help="方格行数")
    ap.add_argument("--mm", type=float, default=25.0, help="每格边长（毫米）")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--paper", default="A4-landscape", choices=["A4-landscape", "A4-portrait"])
    args = ap.parse_args()

    img = make_board(args.squares_x, args.squares_y, args.mm, args.dpi, args.paper)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    name = (f"chessboard_A4_{args.squares_x}x{args.squares_y}_"
            f"{args.mm:.0f}mm_{args.dpi}dpi.png")
    path = OUTPUT_DIR / name
    cv2.imwrite(str(path), img)

    print("=" * 66)
    print("棋盘格已生成")
    print(f"文件      : {rel(path)}")
    print(f"像素      : {img.shape[1]} x {img.shape[0]}")
    print(f"方格      : {args.squares_x} x {args.squares_y} 个，每格 {args.mm:.1f} mm")
    print(f"内角点    : {args.squares_x - 1} x {args.squares_y - 1}  "
          f"-> 标定时用 --cols {args.squares_x - 1} --rows {args.squares_y - 1}")
    print(f"棋盘尺寸  : {args.squares_x * args.mm:.0f} x {args.squares_y * args.mm:.0f} mm")
    print("-" * 66)
    print("打印与核对：")
    print("  1. 打印时选「实际大小 / 100%」，不要「适合页面」")
    print(f"  2. 打印后量 10 个方格的跨度，应为 {10 * args.mm:.0f} mm")
    print("  3. 若实测为 X mm，则标定参数用 --square-mm X/10")
    print("  4. 贴在硬纸板上保证平整，避免反光角度")
    print("=" * 66)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
