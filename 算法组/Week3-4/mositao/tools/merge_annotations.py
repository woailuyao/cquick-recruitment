# -*- coding: utf-8 -*-
"""合并多批预标注结果，按「图像内容指纹」去重。

场景：先用 --every 0.6 抽了一批（202 张），发现带框的图不够 200，
于是用 --every 0.35 又抽了一批（367 张）。两批来自同一批视频，
时间戳必然有重叠，直接合并会引入重复样本，所以这里用图像 MD5 去重。

用法：
    python tools/merge_annotations.py --src dataset/pre2 --dst dataset/pre
    python tools/merge_annotations.py --src dataset/pre2 --dst dataset/pre --include-empty
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import rel          # noqa: E402


def md5_of(path: Path) -> str:
    h = hashlib.md5()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def label_lines(txt: Path) -> int:
    if not txt.exists():
        return 0
    return sum(1 for ln in txt.read_text(encoding="utf-8").splitlines() if ln.strip())


def main() -> int:
    ap = argparse.ArgumentParser(description="合并预标注批次（按内容去重）")
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    ap.add_argument("--include-empty", action="store_true",
                    help="连无框图也一起合并（默认只合并有框的）")
    args = ap.parse_args()

    src = Path(args.src)
    dst = Path(args.dst)
    for p in (src, dst):
        if not p.is_absolute():
            p = ROOT / p
    src, dst = (ROOT / args.src if not Path(args.src).is_absolute() else Path(args.src)), \
               (ROOT / args.dst if not Path(args.dst).is_absolute() else Path(args.dst))

    if not src.exists() or not dst.exists():
        print("源目录或目标目录不存在")
        return 1

    dst_hashes = {md5_of(p) for p in dst.glob("*.png")}
    print("=" * 68)
    print(f"合并 {rel(src)} -> {rel(dst)}")
    print(f"目标已有 {len(list(dst.glob('*.png')))} 张，指纹库 {len(dst_hashes)} 个")
    print("=" * 68)

    added_box = added_empty = skipped_dup = 0
    for img in sorted(src.glob("*.png")):
        txt = img.with_suffix(".txt")
        nb = label_lines(txt)
        if nb == 0 and not args.include_empty:
            continue
        h = md5_of(img)
        if h in dst_hashes:
            skipped_dup += 1
            continue
        dst_hashes.add(h)
        shutil.copy2(img, dst / img.name)
        if txt.exists():
            shutil.copy2(txt, dst / txt.name)
        else:
            (dst / txt.name).write_text("", encoding="utf-8")
        if nb:
            added_box += 1
        else:
            added_empty += 1

    total_img = len(list(dst.glob("*.png")))
    total_with_box = sum(1 for p in dst.glob("*.txt")
                         if p.name != "classes.txt" and label_lines(p) > 0)
    total_boxes = sum(label_lines(p) for p in dst.glob("*.txt") if p.name != "classes.txt")
    print(f"新增有框图 {added_box} 张，新增空框图 {added_empty} 张，"
          f"内容重复跳过 {skipped_dup} 张")
    print("-" * 68)
    print(f"合并后：图片 {total_img} 张，有框 {total_with_box} 张，框总数 {total_boxes} 个")
    if total_with_box < 200:
        print(f"提示：有框图仍不足 200（{total_with_box} 张），可再加密抽帧或补拍远景/遮挡镜头。")
    else:
        print("有框图已超过 200 张，满足细则下限。")
    print("=" * 68)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
