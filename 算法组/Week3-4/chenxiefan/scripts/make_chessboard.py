#!/usr/bin/env python3

import argparse
import os

import cv2
import numpy as np

PAPERS_MM = {
    "a4": (210.0, 297.0),
    "a3": (297.0, 420.0),
}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pattern", nargs=2, type=int, default=[9, 6], help="内角点 横 纵")
    ap.add_argument("--square", type=float, default=18.0, help="方格边长（毫米）")
    ap.add_argument("--paper", choices=sorted(PAPERS_MM), default="a4")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--margin", type=float, default=10.0, help="页边距（毫米）")
    ap.add_argument("--out", default="print/chessboard.png")
    args = ap.parse_args()

    cols, rows = args.pattern
    squares_x, squares_y = cols + 1, rows + 1
    paper_w, paper_h = PAPERS_MM[args.paper]
    board_w = squares_x * args.square
    board_h = squares_y * args.square
    if board_w + 2 * args.margin > paper_w or board_h + 2 * args.margin > paper_h:
        print(f"[错误] {args.paper} 放不下 {squares_x}x{squares_y} 个 {args.square}mm 方格")
        print(f"       纸 {paper_w}x{paper_h} mm，棋盘 {board_w}x{board_h} mm，边距 {args.margin} mm")
        print("       换 --paper a3 / 减小 --square / 减小 --margin")
        return 1

    px_per_mm = args.dpi / 25.4
    page = np.full((int(paper_h * px_per_mm), int(paper_w * px_per_mm)), 255, np.uint8)

    x0 = int((paper_w - board_w) / 2 * px_per_mm)
    y0 = int((paper_h - board_h) / 2 * px_per_mm)
    sq = int(round(args.square * px_per_mm))

    for j in range(squares_y):
        for i in range(squares_x):
            if (i + j) % 2 == 0:
                x1 = x0 + i * sq
                y1 = y0 + j * sq
                page[y1:y1 + sq, x1:x1 + sq] = 0

    out = args.out if os.path.isabs(args.out) else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cv2.imwrite(out, page)

    print(f"已生成 {out}")
    print(f"纸张 {args.paper.upper()}，棋盘 {board_w}x{board_h} mm，方格 {args.square} mm，{args.dpi} dpi")
    print(f"内角点 {cols}x{rows}（用于 calibrate_camera.py --pattern {cols} {rows} --square {args.square}）")

    chk = cv2.findChessboardCorners(page, (cols, rows), None)
    print("自检 findChessboardCorners:", "通过" if chk[0] else "失败（打印后可能需要调整对比度）")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
