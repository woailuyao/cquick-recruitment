# -*- coding: utf-8 -*-
"""体检已标注的类别：用几何判类复核每个框的 class_id，报告（可选）修正。

用途：点击式标注里若误按 1/2/3 强制了类别，那一批框会继承错误类别。本工具按框中心
重新做一次分支分析，把与几何不符的框列出来。

默认只报告，不动文件；加 --apply 才写回（写回前生成 .bak 备份）。

用法（项目根目录）：
    .\\run.bat tools\\audit_marker_labels.py
    .\\run.bat tools\\audit_marker_labels.py --apply
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.common.paths import ROOT, rel          # noqa: E402
import tools.annotate as A                      # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="复核标志标注的类别")
    ap.add_argument("--dir", default="dataset/raw_marker")
    ap.add_argument("--apply", action="store_true", help="把不一致的类别改成几何判定结果")
    args = ap.parse_args()

    d = Path(args.dir)
    if not d.is_absolute():
        d = ROOT / d
    names = A.MARKER_CLASSES

    total = unknown = bad = 0
    rows = []
    cache = {}
    for t in sorted(d.rglob("*.txt")):
        boxes = A.load_boxes(t)
        if not boxes:
            continue
        img_path = t.with_suffix(".png")
        if not img_path.exists():
            img_path = t.with_suffix(".jpg")
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        ctx = cache.setdefault(str(img_path), A.build_context(img))
        changed = False
        for i, b in enumerate(boxes):
            cls, _ = A.classify_at(ctx, b[1] * w, b[2] * h)
            total += 1
            if cls is None:
                unknown += 1
                continue
            if int(b[0]) != cls:
                bad += 1
                rows.append((t, i, names[int(b[0])], names[cls], w, h, b))
                if args.apply:
                    b[0] = cls
                    changed = True
        if changed:
            bak = t.with_suffix(".txt.bak")
            if not bak.exists():
                shutil.copy2(t, bak)
            A.save_boxes(t, boxes)

    print("=" * 74)
    print(f"共复核 {total} 个框；几何判不出 {unknown} 个；与几何不一致 {bad} 个")
    for t, i, was, now, w, h, b in rows:
        print(f"  {rel(t.parent)}/{t.stem}  框#{i}  {was} -> {now}"
              f"  中心 ({b[1] * w:.0f}, {b[2] * h:.0f})")
    if args.apply and bad:
        print(f"\n已按几何判定改写 {bad} 个类别（.bak 备份已留在原目录）")
    elif bad:
        print("\n只报告，未改动。确认无误后加 --apply 写回。")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
