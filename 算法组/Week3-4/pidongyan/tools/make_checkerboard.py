"""生成可打印的棋盘格标定板（A4，尺寸精确到毫米）。

为什么需要它：
  考核细则 2.5 要求用棋盘格完成内参标定。网上随便下的棋盘格经常因为
  「打印时被缩放」「方格尺寸不知道」而让标定结果整体偏大或偏小，
  最典型的现象是 fx 与真实焦距差 3%~10%。这里生成的 PDF 页面尺寸精确为 A4，
  方格边长严格等于 --square-mm，打印时必须选「实际大小 / 100%」。

用法：
  python tools/make_checkerboard.py                      # 默认：10x7 方格(内角点 9x6)，20mm
  python tools/make_checkerboard.py --square-mm 25
  python tools/make_checkerboard.py --no-annotation      # 纯棋盘，不带标尺文字

输出（默认写到 docs/）：
  docs/checkerboard_A4_9x6_20mm.pdf   ← 拿去打印
  docs/checkerboard_A4_9x6_20mm.png   ← 300dpi 预览

打印要点：
  1. 用 Adobe Reader / 系统打印，纸张 A4，缩放选「实际大小 / 100%」，不要「适合页面」；
  2. 打出来用卡尺或直尺量一下最下面那条 100mm 校准尺，不是 100mm 就说明被缩放了；
  3. 贴在硬纸板 / 亚克力上保持平整，软纸弯曲会直接毁掉标定精度；
  4. 哑光纸更好，光面纸反光会在角点检测上产生噪声。
"""
import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[1]
A4_MM = (210.0, 297.0)


def pick_cjk_font():
    """找一个能显示中文的字体；找不到就退回英文标注。"""
    from matplotlib import font_manager
    wanted = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
              "Source Han Sans SC", "WenQuanYi Zen Hei", "PingFang SC"]
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in wanted:
        if name in have:
            return name
    return None


def draw_board(sq_cols, sq_rows, square_mm, annotation, out_stem: Path, cjk):
    board_w = sq_cols * square_mm
    board_h = sq_rows * square_mm
    paper_w, paper_h = A4_MM

    if board_w > paper_w - 10 or board_h > paper_h - 60:
        sys.exit(f"错误：板子 {board_w}x{board_h} mm 放不进 A4（含页边距与标注区）")

    x0 = (paper_w - board_w) / 2
    y0 = (paper_h - board_h) / 2 + (10 if annotation else 0)

    fig = plt.figure(figsize=(paper_w / 25.4, paper_h / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, paper_w)
    ax.set_ylim(0, paper_h)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(Rectangle((0, 0), paper_w, paper_h, facecolor="white", edgecolor="none"))

    # 棋盘格：偶数格黑（保证四角是黑的，SB 检测器更稳）
    for i in range(sq_cols):
        for j in range(sq_rows):
            if (i + j) % 2 == 0:
                ax.add_patch(Rectangle((x0 + i * square_mm, y0 + j * square_mm),
                                       square_mm, square_mm,
                                       facecolor="black", edgecolor="none"))

    if annotation:
        # 100mm 校准尺
        ruler_len = 100.0
        rx = (paper_w - ruler_len) / 2
        ry = y0 - 18
        ax.plot([rx, rx + ruler_len], [ry, ry], color="black", lw=0.8)
        for k in range(11):
            tick = 3 if k % 5 == 0 else 1.6
            ax.plot([rx + k * 10, rx + k * 10], [ry, ry + tick], color="black", lw=0.6)
        ax.text(rx + ruler_len + 3, ry - 0.5, "100 mm", fontsize=6, va="center")

        inner = f"{sq_cols - 1} x {sq_rows - 1}"
        if cjk:
            l1 = f"内角点 {inner}（{sq_cols} x {sq_rows} 方格）  方格 {square_mm:g} mm"
            l2 = "打印请选「实际大小 / 100%」，不要「适合页面」"
        else:
            l1 = f"INNER CORNERS {inner}  ({sq_cols} x {sq_rows} SQUARES)  SQUARE {square_mm:g} mm"
            l2 = "PRINT AT 100% / ACTUAL SIZE - DO NOT FIT TO PAGE"
        ax.text(paper_w / 2, ry - 9, l1, fontsize=8, ha="center", va="center")
        ax.text(paper_w / 2, ry - 15, l2, fontsize=7, ha="center", va="center",
                color="#333333")

    stem = out_stem
    stem.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(stem.with_suffix(".pdf")), format="pdf")
    fig.savefig(str(stem.with_suffix(".png")), dpi=300, format="png")
    plt.close(fig)
    return board_w, board_h


def main():
    ap = argparse.ArgumentParser(description="生成 A4 棋盘格标定板")
    ap.add_argument("--squares", default="10x7",
                    help="方格数量（列x行），默认 10x7 => 内角点 9x6")
    ap.add_argument("--square-mm", type=float, default=20.0, help="方格边长 mm，默认 20")
    ap.add_argument("--no-annotation", action="store_true", help="不画校准尺和文字")
    ap.add_argument("--out-dir", default="docs")
    args = ap.parse_args()

    try:
        c, r = args.squares.lower().split("x")
        sq_cols, sq_rows = int(c), int(r)
    except Exception:
        sys.exit("错误：--squares 格式应为 10x7")

    cjk = pick_cjk_font()
    if cjk:
        plt.rcParams["font.sans-serif"] = [cjk]
        plt.rcParams["axes.unicode_minus"] = False

    out_dir = Path(args.out_dir)
    if not out_dir.is_absolute():
        out_dir = ROOT / out_dir
    stem = out_dir / f"checkerboard_A4_{sq_cols - 1}x{sq_rows - 1}_{args.square_mm:g}mm"

    bw, bh = draw_board(sq_cols, sq_rows, args.square_mm,
                        not args.no_annotation, stem, cjk)

    print("=" * 64)
    print(f"方格数     : {sq_cols} x {sq_rows}")
    print(f"内角点     : {sq_cols - 1} x {sq_rows - 1}   <- 标定脚本用 --cols {sq_cols-1} --rows {sq_rows-1}")
    print(f"方格边长   : {args.square_mm:g} mm   <- 标定脚本用 --square {args.square_mm:g}")
    print(f"板子物理尺寸: {bw:g} x {bh:g} mm（A4 可容纳）")
    print(f"PDF        : {stem.with_suffix('.pdf')}")
    print(f"PNG(300dpi): {stem.with_suffix('.png')}")
    print("=" * 64)
    print("打印后务必用尺量一下底部 100 mm 校准尺：量出来不是 100 mm，说明打印被缩放了，")


if __name__ == "__main__":
    main()
