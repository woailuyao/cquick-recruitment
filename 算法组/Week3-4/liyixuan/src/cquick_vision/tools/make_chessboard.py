#!/usr/bin/env python3
"""生成可以 1:1 打印的棋盘格（A4 横向），标定内参用。

生成后一定要：打印机选"实际大小 / 100%，不要缩放"，打出来用尺子量
下面那条 100 mm 的标尺线，必须是 100 mm，否则 --square 要按实际量到的改。

用法：
  python tools/make_chessboard.py --cols 9 --rows 6 --square 20
  # 生成 calibration/chessboard_9x6_20mm.png（300dpi，A4 横向）
"""
from __future__ import annotations

import argparse
import os

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def _font(px: int):
    """尽量找一个真实字体，找不到就用 PIL 默认位图字体。"""
    for name in ("arial.ttf", "DejaVuSans.ttf", "Helvetica.ttc",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, px)
        except Exception:
            continue
    return ImageFont.load_default()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=9, help="内角点数（列），方格数是 cols+1")
    ap.add_argument("--rows", type=int, default=6, help="内角点数（行），方格数是 rows+1")
    ap.add_argument("--square", type=float, default=20.0, help="方格边长 mm")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    dpi = args.dpi
    mm2px = dpi / 25.4
    sq = args.square * mm2px

    nx, ny = args.cols + 1, args.rows + 1
    board_w, board_h = int(round(nx * sq)), int(round(ny * sq))
    margin = int(round(12 * mm2px))
    ruler_h = int(round(32 * mm2px))
    W = board_w + 2 * margin
    H = board_h + 2 * margin + ruler_h

    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)

    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                x0 = margin + int(round(i * sq))
                y0 = margin + int(round(j * sq))
                d.rectangle([x0, y0, x0 + int(round(sq)), y0 + int(round(sq))],
                            fill="black")

    # 100 mm 标尺：打印后用尺子量，必须正好 100 mm
    y = margin + board_h + int(round(10 * mm2px))
    x0 = margin
    x1 = margin + int(round(100 * mm2px))
    d.line([x0, y, x1, y], fill="black", width=max(2, dpi // 100))
    for xx in (x0, x1):
        d.line([xx, y - int(7 * mm2px), xx, y + int(7 * mm2px)],
               fill="black", width=max(2, dpi // 150))
    f1 = _font(int(4.2 * mm2px))
    f2 = _font(int(3.4 * mm2px))
    d.text((x0, y + int(9 * mm2px)),
           f"this line must be 100 mm after printing (square = {args.square} mm)",
           fill="black", font=f1)
    d.text((x0, y + int(16 * mm2px)),
           f"{args.cols}x{args.rows} inner corners | print at 100% (no scaling)",
           fill="black", font=f2)

    out = args.out or os.path.join(
        ROOT, "calibration", f"chessboard_{args.cols}x{args.rows}_{int(args.square)}mm.png")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    img.save(out, dpi=(dpi, dpi))
    print(f"[board] {out}")
    print(f"[board] 图像 {W}x{H}px @ {dpi}dpi  ->  "
          f"{W / mm2px:.0f} x {H / mm2px:.0f} mm，A4 横向可打印")
    print(f"[board] 打印后用尺子量标尺线，必须 = 100 mm；"
          f"若量到 X mm（比如 97），就把 --square 改成 {args.square}*X/100（比如 19.4）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
