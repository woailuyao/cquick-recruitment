# -*- coding: utf-8 -*-
"""标注进度速查：随时看「还剩多少张、哪些还没框」。

用法：
    python tools/annot_progress.py
    python tools/annot_progress.py --dir dataset/pre --list 30
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import rel          # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="标注进度速查")
    ap.add_argument("--dir", default="dataset/pre")
    ap.add_argument("--list", type=int, default=15, help="列出多少个空框文件名")
    args = ap.parse_args()

    d = Path(args.dir)
    if not d.is_absolute():
        d = ROOT / d
    if not d.exists():
        print(f"目录不存在：{rel(d)}")
        return 1

    imgs = sorted(p for p in d.glob("*.png"))
    labels = {p.stem: p for p in d.glob("*.txt") if p.name != "classes.txt"}
    xmls = list(d.glob("*.xml"))

    total_box = 0
    empty, filled = [], []
    for img in imgs:
        lab = labels.get(img.stem)
        if lab is None:
            empty.append(img.stem)
            continue
        lines = [ln for ln in lab.read_text(encoding="utf-8").splitlines() if ln.strip()]
        total_box += len(lines)
        (filled if lines else empty).append(img.stem)

    print("=" * 66)
    print(f"目录     : {rel(d)}")
    print(f"图片     : {len(imgs)} 张")
    print(f"有框     : {len(filled)} 张")
    print(f"无框     : {len(empty)} 张")
    print(f"框总数   : {total_box} 个")
    print(f"XML 残留 : {len(xmls)} 个" + ("  ← 格式选错了，应为 YOLO" if xmls else "  ✓ 格式正确"))
    n_ready = sum(1 for s in filled) + sum(1 for s in empty if labels.get(s))
    print(f"已处理   : {len(filled) + (len(empty) - sum(1 for s in empty if labels.get(s) is None))}"
          f" 张（含确认无球而留空的）")
    print("-" * 66)
    if empty:
        print(f"仍有 {len(empty)} 张没有标签文件（LabelImg 里翻到就会自动建）：")
        for s in empty[: args.list]:
            print("   " + s)
        if len(empty) > args.list:
            print(f"   ... 另有 {len(empty) - args.list} 张")
    print("=" * 66)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
