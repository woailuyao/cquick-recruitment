# -*- coding: utf-8 -*-
"""把误检 / 漏检 / 正确案例拼成一张图，录屏讲解时一屏看全。

`outputs/cases_ball/` 里的文件是 `tools/evaluate.py` 导出的单张案例图：
    fn_*.png  漏检（人工标了、模型没检出；橙框是人工标注）
    fp_*.png  误检（模型框出来了、人工没标；红框是模型的框）
    tp_*.png  正确（绿框）
单张翻看既费事，录屏时还要来回切窗口。本脚本把它们拼成一张带中文标题的 PNG。

用法（项目根目录）：
    .\\run.bat tools\\case_sheet.py
    .\\run.bat tools\\case_sheet.py --dir outputs\\cases_ball --cols 3

输出：outputs/cases_ball/case_sheet_<HHMMSS>.png
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

KIND = {
    "fn": ("漏检", (255, 150, 40), "人工标了、模型没检出"),
    "fp": ("误检", (230, 60, 60), "模型框出来了、人工没标"),
    "tp": ("正确", (60, 200, 90), "检出框与人工标注一致"),
}
FONT_CANDIDATES = [
    r"C:\Windows\Fonts\msyh.ttc",
    r"C:\Windows\Fonts\msyhbd.ttc",
    r"C:\Windows\Fonts\simhei.ttf",
]


def load_font(size: int) -> ImageFont.FreeTypeFont:
    for p in FONT_CANDIDATES:
        if Path(p).exists():
            try:
                return ImageFont.truetype(p, size)
            except OSError:
                continue
    return ImageFont.load_default(size)


def collect(img_dir: Path) -> list[tuple[str, Path]]:
    order = {"fn": 0, "fp": 1, "tp": 2}
    items = []
    for p in sorted(img_dir.glob("*.png")):
        key = p.name[:2]
        if key in order and not p.name.startswith("case_sheet"):
            items.append((key, p))
    items.sort(key=lambda t: (order[t[0]], t[1].name))
    return items


def main() -> int:
    ap = argparse.ArgumentParser(description="案例图拼版")
    ap.add_argument("--dir", default=str(ROOT / "outputs" / "cases_ball"))
    ap.add_argument("--cols", type=int, default=3)
    ap.add_argument("--cell-w", type=int, default=640)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    img_dir = Path(args.dir)
    items = collect(img_dir)
    if not items:
        print(f"没找到案例图：{img_dir}")
        return 1

    cell_w = args.cell_w
    cell_h = int(cell_w * 720 / 1280)
    pad, label_h, head_h, foot_h = 12, 30, 56, 62
    cols = max(1, args.cols)
    rows = (len(items) + cols - 1) // cols
    W = pad + cols * (cell_w + pad)
    H = head_h + pad + rows * (cell_h + label_h + pad) + foot_h

    canvas = Image.new("RGB", (W, H), (26, 26, 30))
    draw = ImageDraw.Draw(canvas)
    f_title, f_label, f_note = load_font(26), load_font(19), load_font(17)

    draw.text((pad, 14), "任务一 检测案例（漏检/正确来自独立测试集，误检来自真机开放场景）",
              font=f_title, fill=(240, 240, 240))
    draw.text((W - 320, 22), datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
              font=f_note, fill=(150, 150, 160))

    for i, (key, path) in enumerate(items):
        name, color, _desc = KIND[key]
        r, c = divmod(i, cols)
        x0 = pad + c * (cell_w + pad)
        y0 = head_h + pad + r * (cell_h + label_h + pad)
        try:
            img = Image.open(path).convert("RGB").resize((cell_w, cell_h), Image.LANCZOS)
        except Exception as exc:                                    # noqa: BLE001
            print(f"跳过 {path.name}: {type(exc).__name__}")
            continue
        canvas.paste(img, (x0, y0))
        draw.rectangle([x0, y0, x0 + cell_w - 1, y0 + cell_h - 1], outline=(70, 70, 78), width=1)
        draw.rectangle([x0, y0 + cell_h, x0 + cell_w - 1, y0 + cell_h + label_h - 1],
                       fill=(38, 38, 44))
        draw.text((x0 + 8, y0 + cell_h + 5), f"{name}  {path.name[:26]}",
                  font=f_label, fill=color)

    # 图例说明
    y = H - foot_h + 8
    x = pad
    for key in ("fn", "fp", "tp"):
        name, color, desc = KIND[key]
        if not any(k == key for k, _ in items):
            continue
        draw.rectangle([x, y + 4, x + 14, y + 18], outline=color, width=3)
        draw.text((x + 22, y), f"{name}：{desc}", font=f_note, fill=(200, 200, 205))
        x += 360
    draw.text((pad, H - 28), "橙框＝人工标注、红框＝模型误报、绿框＝正确检出",
              font=f_note, fill=(150, 150, 160))

    out = Path(args.out) if args.out else img_dir / f"case_sheet_{datetime.now():%H%M%S}.png"
    canvas.save(out)
    print(f"拼版已保存：{out}  （{W}×{H}，{len(items)} 张）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
