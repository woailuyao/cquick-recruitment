# -*- coding: utf-8 -*-
"""
生成可打印的棋盘格标定板（PDF + PNG）

为什么自己做：标定板要求**方格边长精确已知**，随便找张图打印会因为在打印对话框里
被"适应页面"缩放过而尺寸失真，标定出来的内参就会出现系统性偏差。

本脚本按毫米精确排版，输出的 PDF 只要**100% 原尺寸打印**（不要"适应页面"），
方格边长就是标签上写的值。底部还带一把 100 mm 校验尺，
打印完拿直尺量一下，确认是 100 mm 再开始标定。

用法：
    # A4 + 9x6 内角点 + 20mm 方格（默认，最常用）
    python scripts/make_chessboard.py

    # 自定义
    python scripts/make_chessboard.py --pattern 9x6 --square-mm 20 --page a4

    # 也可以出信纸尺寸
    python scripts/make_chessboard.py --page letter

输出到 calibration/board/ 目录下。
"""

from __future__ import annotations

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTDIR = os.path.join(ROOT, "calibration", "board")

PAGES = {           # 单位 mm
    "a4": (210.0, 297.0),
    "letter": (215.9, 279.4),
    "a3": (297.0, 420.0),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pattern", default="9x6", help="内角点数，宽x高")
    ap.add_argument("--square-mm", type=float, default=18.0,
                    help="方格边长，毫米。A4 + 9x6 内角点默认 18mm："
                         "10 个方格宽 = 180mm，正好留出打印机不可打印边距")
    ap.add_argument("--page", default="a4", choices=list(PAGES))
    ap.add_argument("--margin-mm", type=float, default=10.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    cols, rows = (int(v) for v in args.pattern.lower().split("x"))
    sq = args.square_mm
    page_w, page_h = PAGES[args.page]

    # 方块数 = 内角点数 + 1
    n_sq_x, n_sq_y = cols + 1, rows + 1
    board_w = n_sq_x * sq
    board_h = n_sq_y * sq

    if board_w > page_w - 2 * args.margin_mm or board_h > page_h - 2 * args.margin_mm:
        print(f"[错误] {n_sq_x}x{n_sq_y} 个方格 x {sq}mm = "
              f"{board_w:.0f}x{board_h:.0f}mm，放不进 {args.page} "
              f"({page_w:.0f}x{page_h:.0f}mm，边距 {args.margin_mm}mm)")
        max_sq = min((page_w - 2 * args.margin_mm) / n_sq_x,
                     (page_h - 2 * args.margin_mm) / n_sq_y)
        print(f"         建议 --square-mm {int(max_sq)} 或更小")
        return 1

    os.makedirs(OUTDIR, exist_ok=True)

    # 按毫米建图：figsize 用英寸，坐标轴用毫米
    fig = plt.figure(figsize=(page_w / 25.4, page_h / 25.4), dpi=300)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, page_w)
    ax.set_ylim(0, page_h)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(Rectangle((0, 0), page_w, page_h, facecolor="white", edgecolor="none"))

    # 棋盘格居中，略微上移给底部校验尺留位置
    x0 = (page_w - board_w) / 2.0
    y0 = (page_h - board_h) / 2.0 + 8.0

    # 重要：留出白色静区（quiet zone），否则检测角点容易失败
    quiet = sq * 0.6
    ax.add_patch(Rectangle((x0 - quiet, y0 - quiet),
                           board_w + 2 * quiet, board_h + 2 * quiet,
                           facecolor="white", edgecolor="none"))

    for i in range(n_sq_x):
        for j in range(n_sq_y):
            if (i + j) % 2 == 0:
                ax.add_patch(Rectangle((x0 + i * sq, y0 + j * sq), sq, sq,
                                       facecolor="black", edgecolor="none"))

    # 外框（细线，方便确认整块板都在画面里）
    ax.add_patch(Rectangle((x0, y0), board_w, board_h,
                           facecolor="none", edgecolor="black", linewidth=0.4))

    # 底部 100 mm 校验尺
    ruler_y = y0 - quiet - 14.0
    ruler_x = (page_w - 100.0) / 2.0
    ax.plot([ruler_x, ruler_x + 100.0], [ruler_y, ruler_y], color="black", linewidth=0.8)
    for k in range(11):
        xx = ruler_x + k * 10.0
        h = 4.0 if k % 5 == 0 else 2.2
        ax.plot([xx, xx], [ruler_y, ruler_y + h], color="black", linewidth=0.6)
    ax.text(ruler_x + 50.0, ruler_y - 5.0,
            "100 mm - verify with a real ruler (print at 100%, no scaling)",
            ha="center", va="top", fontsize=6)

    # 标题与参数
    ax.text(page_w / 2.0, y0 + board_h + quiet + 12.0,
            f"Chessboard  inner corners {cols} x {rows}   squares {n_sq_x} x {n_sq_y}   "
            f"square = {sq:g} mm   board = {board_w:g} x {board_h:g} mm",
            ha="center", va="bottom", fontsize=8)
    ax.text(page_w / 2.0, 8.0,
            "Print at 100% scale (Actual size / None). Do NOT use Fit-to-page. "
            "Mount on a rigid flat board.",
            ha="center", va="bottom", fontsize=7)

    stem = args.out or os.path.join(
        OUTDIR, f"chessboard_{cols}x{rows}_{sq:g}mm_{args.page}")
    pdf = stem + ".pdf"
    png = stem + ".png"
    fig.savefig(pdf, format="pdf")
    fig.savefig(png, dpi=300)
    plt.close(fig)

    print("=" * 66)
    print("棋盘格已生成")
    print("=" * 66)
    print(f"  内角点     : {cols} x {rows}")
    print(f"  方格数     : {n_sq_x} x {n_sq_y}")
    print(f"  方格边长   : {sq:g} mm  （标定时要用这个值）")
    print(f"  板面尺寸   : {board_w:g} x {board_h:g} mm")
    print(f"  PDF -> {pdf}")
    print(f"  PNG -> {png}")
    print()
    print("打印要点：")
    print("  1. 打印对话框里选【实际大小 / 100% / 无缩放】，绝不能选『适应页面』；")
    print("  2. 打完用直尺量底部那条校验尺，必须是 100 mm，差 1 mm 就会让焦距偏 1%；")
    print("  3. 一定贴在硬板上（打印纸单纯拿着会弯，弯了标定必偏）；")
    print("  4. A4 纸尽量选哑光，光面纸在灯光下会反光，角点检测会失败。")
    print()
    print("标定图采集：")
    print(f"  .\\.venv\\Scripts\\python.exe scripts\\calibrate_capture.py "
          f"--camera 1 --pattern {cols}x{rows}")
    print("标定计算：")
    print(f"  .\\.venv\\Scripts\\python.exe scripts\\calibrate_camera.py "
          f"--images calib_images --pattern {cols}x{rows} --square {sq:g}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
