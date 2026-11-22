"""把标签框统一成固定大小（以原框中心为中心），修复"大框/小框混着训"的问题。

问题：旧的 146 张图里 140 张的框占画面 30~60%，新标的 T1/X1 只有 10~15%。
      两种尺度混在一起训练，YOLO 会往大框那边塌，推理时满屏乱框（实测就是这样）。

做法：只把**比目标尺寸大的框**缩小到目标边长，中心保持不变；小框不动。
      默认先备份到 dataset/labels_backup_<日期>/。

    python scripts/normalize_labels.py --dry-run          # 先看统计
    python scripts/normalize_labels.py --size 220 --apply # 真的改
"""

from __future__ import annotations

import argparse
import datetime
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))
DEFAULT_DIR = ROOT / "dataset" / "images" / "selected"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--labels", default=str(DEFAULT_DIR), help="图片/标签所在目录")
    ap.add_argument("--size", type=float, default=220.0,
                    help="目标框边长（像素）；只缩小，不放大")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    lab_dir = Path(args.labels)
    txts = sorted(lab_dir.glob("*.txt"))
    if not txts:
        raise SystemExit(f"no .txt in {lab_dir}")

    backup = lab_dir.parent.parent / f"labels_backup_{datetime.date.today():%Y%m%d}"
    if args.apply:
        if not backup.exists():
            shutil.copytree(lab_dir, backup,
                            ignore=shutil.ignore_patterns("*.jpg", "*.png", "*.bmp"))
            print(f"已备份原始标签到 {backup}")
        else:
            print(f"备份已存在: {backup}")

    W, H = args.width, args.height
    half_w, half_h = args.size / 2 / W, args.size / 2 / H

    changed = 0
    total = 0
    areas_before, areas_after = [], []
    for t in txts:
        lines = [l for l in t.read_text(encoding="utf-8").splitlines() if l.strip()]
        if not lines:
            continue
        out = []
        modified = False
        for line in lines:
            p = line.split()
            if len(p) != 5:
                out.append(line)
                continue
            cls = int(p[0])
            cx, cy, bw, bh = map(float, p[1:5])
            total += 1
            areas_before.append(bw * bh)
            if bw > 2 * half_w or bh > 2 * half_h:
                bw = min(bw, 2 * half_w)
                bh = min(bh, 2 * half_h)
                # 保证框不出界
                cx = min(max(cx, bw / 2 + 0.002), 1 - bw / 2 - 0.002)
                cy = min(max(cy, bh / 2 + 0.002), 1 - bh / 2 - 0.002)
                modified = True
            areas_after.append(bw * bh)
            out.append(f"{cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
        if modified:
            changed += 1
            if args.apply:
                t.write_text("\n".join(out) + "\n", encoding="utf-8")

    import statistics
    print(f"标签文件 : {len(txts)}   框总数: {total}")
    print(f"会被改的 : {changed} 个文件")
    print(f"框面积占比 中位数: 改前 {statistics.median(areas_before):.3f} "
          f"-> 改后 {statistics.median(areas_after):.3f}")
    print(f"最长边换算: 目标 {args.size:.0f}px = {args.size/W:.2f} 图宽")

    if not args.apply:
        print("\n（加 --apply 才会真的写回；会先备份）")
        return
    print("已写回")


if __name__ == "__main__":
    main()