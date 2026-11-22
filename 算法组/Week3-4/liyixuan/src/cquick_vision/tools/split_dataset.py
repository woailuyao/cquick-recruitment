#!/usr/bin/env python3
"""按"视频片段"划分 train/val/test，并把图片和同名标签一起拷进 YOLO 目录。

为什么必须按片段分：同一段视频的相邻帧几乎一样，如果随机打散到三个集合，
测试集里会混进训练集的"近亲"，指标会虚高，评审会扣分。

约定目录（每段视频一个子文件夹，图片和同名 .txt 放一起）：
  raw/frames/
    ball_v01/  ball_v01_00000.jpg  ball_v01_00000.txt  ...
    ball_v02/  ...
    ball_v05/  ...

用法：
  python tools/split_dataset.py --src raw/frames --out soccer_dataset \
      --val ball_v05 --test ball_v06
  # 除 val/test 列出的视频外，其余全部进 train
"""
from __future__ import annotations

import argparse
import os
import shutil
from collections import Counter


def split_by_time(args, exts):
    """只有一段视频时：按文件名（即时间）顺序切成 train/val/test。

    比随机打散好：随即打散会让相邻帧同时出现在 train 和 test 里，
    指标会虚高；按时间块切分，val/test 是视频里完全不同的时间段。
    """
    ratios = [float(x) for x in args.time_split.split(",")]
    if len(ratios) != 3 or abs(sum(ratios) - 1.0) > 0.02:
        print("[error] --time-split 需要三个数且和为 1，例如 0.70,0.15,0.15")
        return 2

    files = []
    direct = [f for f in os.listdir(args.src) if f.lower().endswith(exts)]
    if direct:
        files = [(args.src, f) for f in sorted(direct)]
    else:
        for d in sorted(os.listdir(args.src)):
            sub = os.path.join(args.src, d)
            if not os.path.isdir(sub):
                continue
            for f in sorted(os.listdir(sub)):
                if f.lower().endswith(exts):
                    files.append((sub, f))
    if not files:
        print(f"[error] {args.src} 里没有图片")
        return 2

    n = len(files)
    n_tr = int(round(n * ratios[0]))
    n_va = int(round(n * ratios[1]))
    n_tr = max(1, min(n - 2, n_tr))
    n_va = max(1, min(n - n_tr - 1, n_va))
    bounds = {"train": (0, n_tr), "val": (n_tr, n_tr + n_va), "test": (n_tr + n_va, n)}
    print(f"[split] 按时间顺序切分 {n} 张（非随机打散）：")
    for k, (a, b) in bounds.items():
        print(f"  {k:5s}: {b - a:3d} 张  （第 {a}~{b-1} 张）")

    stats = Counter()
    empty = missing = 0
    for split, (a, b) in bounds.items():
        img_out = os.path.join(args.out, "images", split)
        lbl_out = os.path.join(args.out, "labels", split)
        if not args.dry_run:
            os.makedirs(img_out, exist_ok=True)
            os.makedirs(lbl_out, exist_ok=True)
        for src_dir, f in files[a:b]:
            stem = os.path.splitext(f)[0]
            lbl = os.path.join(src_dir, stem + ".txt")
            if not os.path.exists(lbl):
                missing += 1
                continue
            with open(lbl, "r", encoding="utf-8") as fh:
                body = fh.read().strip()
            if not body:
                empty += 1
                continue
            if not args.dry_run:
                shutil.copy2(os.path.join(src_dir, f), os.path.join(img_out, f))
                shutil.copy2(lbl, os.path.join(lbl_out, stem + ".txt"))
            stats[split] += 1

    print(f"\n[split] 实际拷入：train {stats['train']} / val {stats['val']} / "
          f"test {stats['test']}（跳过未标 {missing}、空标签 {empty}）")
    if args.dry_run:
        return 0
    if stats["val"] == 0 or stats["test"] == 0:
        print("[warn] val 或 test 为空，无法评估")

    names = [x.strip() for x in args.names.split(",") if x.strip()]
    yaml_path = args.yaml or os.path.join(os.path.abspath(args.out), "data.yaml")
    abs_out = os.path.abspath(args.out).replace("\\", "/")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(f"path: {abs_out}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("test: images/test\n")
        f.write(f"nc: {len(names)}\n")
        f.write("names: [ " + ", ".join(f'"{x}"' for x in names) + " ]\n")
    print(f"[done] data.yaml 已写好：{yaml_path}")
    print(f"[done] 直接训练：")
    print(f"       yolo detect train model=yolov8n.pt data=\"{yaml_path}\" "
          f"epochs=20 imgsz=640 batch=16 device=cpu")
    return 0


def split_random(args, exts):
    """随机划分（用于只有一段视频、且画面变化很大的情况）。

    注意：相邻帧会被分到不同集合，指标会偏乐观，README 必须如实说明。
    """
    import random
    ratios = [float(x) for x in args.random_split.split(",")]
    if len(ratios) != 3 or abs(sum(ratios) - 1.0) > 0.02:
        print("[error] --random-split 需要三个数且和为 1，例如 0.80,0.10,0.10")
        return 2

    files = []
    direct = [f for f in os.listdir(args.src) if f.lower().endswith(exts)]
    if direct:
        files = [(args.src, f) for f in sorted(direct)]
    else:
        for d in sorted(os.listdir(args.src)):
            sub = os.path.join(args.src, d)
            if not os.path.isdir(sub):
                continue
            for f in sorted(os.listdir(sub)):
                if f.lower().endswith(exts):
                    files.append((sub, f))

    # 只保留有标签的（没标的不参与）
    usable = []
    for src_dir, f in files:
        stem = os.path.splitext(f)[0]
        lbl = os.path.join(src_dir, stem + ".txt")
        if not os.path.exists(lbl):
            continue
        with open(lbl, "r", encoding="utf-8") as fh:
            if not fh.read().strip():
                continue
        usable.append((src_dir, f))

    random.Random(args.seed).shuffle(usable)
    n = len(usable)
    n_tr = max(1, int(round(n * ratios[0])))
    n_va = max(1, int(round(n * ratios[1])))
    if n_tr + n_va >= n:
        n_va = max(1, n - n_tr - 1)
    parts = {"train": usable[:n_tr],
             "val": usable[n_tr:n_tr + n_va],
             "test": usable[n_tr + n_va:]}
    print(f"[split] 随机划分 {n} 张（seed={args.seed}，"
          f"注意：相邻帧会跨集合，指标偏乐观）")
    for k, v in parts.items():
        print(f"  {k:5s}: {len(v):3d} 张")

    for split, items in parts.items():
        img_out = os.path.join(args.out, "images", split)
        lbl_out = os.path.join(args.out, "labels", split)
        if not args.dry_run:
            os.makedirs(img_out, exist_ok=True)
            os.makedirs(lbl_out, exist_ok=True)
        for src_dir, f in items:
            stem = os.path.splitext(f)[0]
            if args.dry_run:
                continue
            shutil.copy2(os.path.join(src_dir, f), os.path.join(img_out, f))
            shutil.copy2(os.path.join(src_dir, stem + ".txt"),
                         os.path.join(lbl_out, stem + ".txt"))
    if args.dry_run:
        return 0

    names = [x.strip() for x in args.names.split(",") if x.strip()]
    yaml_path = args.yaml or os.path.join(os.path.abspath(args.out), "data.yaml")
    abs_out = os.path.abspath(args.out).replace("\\", "/")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(f"path: {abs_out}\n")
        f.write("train: images/train\nval: images/val\ntest: images/test\n")
        f.write(f"nc: {len(names)}\n")
        f.write("names: [ " + ", ".join(f'"{x}"' for x in names) + " ]\n")
    print(f"[done] data.yaml 已写好：{yaml_path}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="每个子文件夹是一段视频的抽帧")
    ap.add_argument("--out", required=True, help="输出 YOLO 数据集根目录")
    ap.add_argument("--val", default="", help="逗号分隔，进 val 的视频文件夹名")
    ap.add_argument("--test", default="", help="逗号分隔，进 test 的视频文件夹名")
    ap.add_argument("--exts", default=".jpg,.jpeg,.png")
    ap.add_argument("--copy", action="store_true", default=True)
    ap.add_argument("--names", default="ball",
                    help="类别名，逗号分隔；足球数据集就是 ball")
    ap.add_argument("--time-split", default=None,
                    help="只有一段视频时按时间顺序切分，如 0.70,0.15,0.15")
    ap.add_argument("--random-split", default=None,
                    help="随机划分，如 0.80,0.10,0.10（单段视频且画面变化大时用）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--yaml", default=None,
                    help="生成的 data.yaml 路径，默认写到 <out>/data.yaml")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    exts = tuple(e.strip().lower() for e in args.exts.split(",") if e.strip())
    val_set = {v.strip() for v in args.val.split(",") if v.strip()}
    test_set = {t.strip() for t in args.test.split(",") if t.strip()}

    if not os.path.isdir(args.src):
        print(f"[error] 找不到 {args.src}")
        return 2

    # ---------------- 只有一段视频时的按时间切分 ----------------
    if args.time_split:
        return split_by_time(args, exts)
    if args.random_split:
        return split_random(args, exts)

    videos = sorted(d for d in os.listdir(args.src)
                    if os.path.isdir(os.path.join(args.src, d)))
    if not videos:
        print(f"[error] {args.src} 下没有子文件夹（每个子文件夹应是一段视频的抽帧）")
        return 2

    unknown = (val_set | test_set) - set(videos)
    if unknown:
        print(f"[error] --val/--test 里这些名字找不到对应文件夹: {sorted(unknown)}")
        print(f"        实际存在: {videos}")
        return 2
    if not (val_set or test_set):
        print("[error] 至少要指定 --val 或 --test，否则全部进 train，无法评估")
        return 2

    splits = {}
    for v in videos:
        splits[v] = "val" if v in val_set else ("test" if v in test_set else "train")

    print("[split] 分配结果（按视频片段，不按帧打散）")
    for v in videos:
        n = sum(1 for f in os.listdir(os.path.join(args.src, v))
                if f.lower().endswith(exts))
        print(f"  {v:24s} -> {splits[v]:5s}  {n} 张")

    stats: Counter = Counter()
    missing = []
    empty = []
    multi = []

    for v in videos:
        split = splits[v]
        sdir = os.path.join(args.src, v)
        img_out = os.path.join(args.out, "images", split)
        lbl_out = os.path.join(args.out, "labels", split)
        if not args.dry_run:
            os.makedirs(img_out, exist_ok=True)
            os.makedirs(lbl_out, exist_ok=True)

        for f in sorted(os.listdir(sdir)):
            if not f.lower().endswith(exts):
                continue
            stem = os.path.splitext(f)[0]
            img_src = os.path.join(sdir, f)
            lbl_src = os.path.join(sdir, stem + ".txt")
            if not os.path.exists(lbl_src):
                missing.append(img_src)
                # 关键：没有标签的图不能拷进数据集，否则 YOLO 会把它当"背景图"训练，
                # 反而教坏模型。宁可少几张。
                continue
            with open(lbl_src, "r", encoding="utf-8") as fh:
                lines = [ln for ln in fh.read().splitlines() if ln.strip()]
            if not lines:
                # 空标签 = 图上没框。绝大多数情况下是"还没标"，不是真的没有球。
                # 拷进去会被 YOLO 当成背景图训练，反而教坏模型，所以跳过。
                empty.append(lbl_src)
                continue
            if len(lines) > 1:
                multi.append((lbl_src, len(lines)))
            if args.dry_run:
                stats[split] += 1
                continue
            shutil.copy2(img_src, os.path.join(img_out, f))
            shutil.copy2(lbl_src, os.path.join(lbl_out, stem + ".txt"))
            stats[split] += 1

    print("\n[split] 统计")
    total = sum(stats.values())
    for s in ("train", "val", "test"):
        n = stats.get(s, 0)
        pct = 100.0 * n / total if total else 0.0
        print(f"  {s:5s}: {n:4d} 张 ({pct:5.1f}%)")
    print(f"  合计 : {total:4d} 张")

    if missing:
        print(f"\n[warn] {len(missing)} 张图片没有对应标签（未标注），已跳过拷贝：")
        for m in missing[:5]:
            print("   ", m)
        if len(missing) > 5:
            print(f"    ... 还有 {len(missing) - 5} 张")
    if empty:
        print(f"\n[info] {len(empty)} 个标签文件为空（纯背景图，YOLO 允许，但别太多）")
    if multi:
        print(f"\n[info] {len(multi)} 张图有多个框（足球检测应为 1 个，检查是否重复标注）：")
        for m, n in multi[:5]:
            print(f"    {m}  {n} 个框")

    if stats.get("val", 0) == 0 or stats.get("test", 0) == 0:
        print("\n[warn] val 或 test 为空，无法评估")
    if args.dry_run:
        print("\n[dry-run] 只统计，没有拷贝文件")
        return 0

    # 顺手把 data.yaml 写出来，省得手动建文件
    names = [x.strip() for x in args.names.split(",") if x.strip()]
    yaml_path = args.yaml or os.path.join(os.path.abspath(args.out), "data.yaml")
    abs_out = os.path.abspath(args.out).replace("\\", "/")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(f"path: {abs_out}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("test: images/test\n")
        f.write(f"nc: {len(names)}\n")
        f.write("names: [ " + ", ".join(f'"{n}"' for n in names) + " ]\n")

    print(f"\n[done] 输出到 {abs_out}/images/{{train,val,test}} 和 labels/{{...}}")
    print(f"[done] data.yaml 已写好：{yaml_path}")
    print(f"[done] 直接训练：")
    print(f"       yolo detect train model=yolov8n.pt data=\"{yaml_path}\" "
          f"epochs=20 imgsz=640 batch=16 device=cpu")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
