# -*- coding: utf-8 -*-
"""把标注框规整成方形 —— 手动拖框难免画扁，这里统一补齐短边

为什么需要
    标志点的语义是「白线在这里接头」，判断 L/T/X 靠的是"线在框里的走向"：
    L 是两条都在此终止、T 是一条撞在连续线上、X 是穿过。
    如果框画成 40x17 的扁条，竖线几乎全在框外，模型就看不见"到没到头"，
    只能学到"横线上有块亮斑"，容易把普通线上的任意点误检成接头。

    YOLO 的框只负责定位，但框内图案才是分类依据，所以框要足够大、
    把接头的几条臂都包进去。这里统一把短边扩到长边（保持中心不变）。

用法
    python normalize_boxes.py --dry              # 只看会改什么
    python normalize_boxes.py                    # 实际写盘（自动备份原 txt）
    python normalize_boxes.py --size 44          # 强制所有框都是 44x44
    python normalize_boxes.py --raw raw_mark --seg T_2   # 只改一段
    python normalize_boxes.py --min 0            # 允许缩小（默认只放大不缩小）
"""
import argparse
import os
import shutil

RAW = "raw_mark"
BACKUP = "_probe/labels_before_normalize"


def read_yolo(path):
    out = []
    for line in open(path, encoding="utf-8"):
        p = line.split()
        if len(p) < 5:
            continue
        out.append((int(float(p[0])), float(p[1]), float(p[2]), float(p[3]), float(p[4])))
    return out


def write_yolo(path, boxes):
    with open(path, "w", encoding="utf-8") as f:
        for c, cx, cy, w, h in boxes:
            f.write(f"{c} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default=RAW)
    ap.add_argument("--seg", default=None, help="只处理这一段")
    ap.add_argument("--size", type=float, default=None,
                    help="强制边长（像素@640x480）。不给就按每个框自己的长边")
    ap.add_argument("--min", dest="min_size", type=float, default=0.0,
                    help="最小边长下限（默认 0）")
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()

    segs = [args.seg] if args.seg else sorted(
        d for d in os.listdir(args.raw) if os.path.isdir(os.path.join(args.raw, d)))

    total_txt = total_box = changed = 0
    detail = []

    for seg in segs:
        p = os.path.join(args.raw, seg)
        if not os.path.isdir(p):
            print(f"[skip] {p} 不存在")
            continue
        for fn in sorted(os.listdir(p)):
            if not fn.lower().endswith(".txt") or fn.lower() == "classes.txt":
                continue
            tp = os.path.join(p, fn)
            boxes = read_yolo(tp)
            if not boxes:
                continue
            total_txt += 1
            new = []
            touched = 0
            for (c, cx, cy, w, h) in boxes:
                total_box += 1
                # 换算到像素（标注按 640x480 归一化）
                pw, ph = w * 640.0, h * 480.0
                base = args.size if args.size else max(pw, ph)
                if base < max(pw, ph):
                    base = max(pw, ph)          # 默认只放大不缩小
                side = max(base, args.min_size)
                nw, nh = side / 640.0, side / 480.0
                if abs(nw - w) > 1e-6 or abs(nh - h) > 1e-6:
                    touched += 1
                # 保持中心，超界则平移回画面内
                ncx = min(max(cx, nw / 2), 1 - nw / 2)
                ncy = min(max(cy, nh / 2), 1 - nh / 2)
                new.append((c, ncx, ncy, nw, nh))
                detail.append((seg, fn, pw, ph, side))
            if touched:
                changed += 1
                if not args.dry:
                    os.makedirs(os.path.join(BACKUP, seg), exist_ok=True)
                    src = os.path.join(BACKUP, seg, fn)
                    if not os.path.exists(src):
                        shutil.copy2(tp, src)
                    write_yolo(tp, new)

    print(f"{'[干跑] ' if args.dry else ''}扫描 {total_txt} 个标注文件 / {total_box} 个框，需修改 {changed} 个文件")
    if detail:
        print("\n样例（像素，640x480 基准）：")
        for seg, fn, pw, ph, side in detail[:8]:
            print(f"  {seg}/{fn:<14} 原 {pw:5.1f}x{ph:5.1f}  ->  {side:5.1f}x{side:5.1f}")
    if not args.dry and changed:
        print(f"\n原标注已备份到 {BACKUP}/")
    if args.dry:
        print("\n加 --dry 都没写盘，去掉就会实际修改")


if __name__ == "__main__":
    main()
