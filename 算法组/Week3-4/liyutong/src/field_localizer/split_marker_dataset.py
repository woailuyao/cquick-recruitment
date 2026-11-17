# -*- coding: utf-8 -*-
"""
标志点数据集划分 —— 同样按「视频段」整段划分，绝不随机打散单帧

为什么不能随机打散
    同一段视频里相邻帧几乎一样，随机打散会让几乎相同的图同时出现在 train 和 val，
    val 指标虚高，实际换到新视角就崩。整段划分才测得准。

用法
    python split_marker_dataset.py
    python split_marker_dataset.py --dry     # 只看怎么分，不复制文件

前提
    raw_mark/<段>/*.jpg + 同名 .txt（label_tool.py --cls 产出，类别 L=0 T=1 X=2）
    neg_* 段是背景样本（无标志点），会给它们补空 txt，训练时当负样本用

输出
    marker_dataset/images/{train,val,test}
    marker_dataset/labels/{train,val,test}
    marker_dataset/data.yaml
"""
import argparse
import os
import shutil

RAW = "raw_mark"
DST = "marker_dataset"
CLASSES = ["L", "T", "X"]
SPLITS = ["train", "val", "test"]

# 段数够多时每 6 段一轮的模式：约 67% / 17% / 17%，且让 val/test 分散在不同时间段，
# 避免「val 全是同一段视频的近似帧」
PATTERN = ["train", "train", "val", "train", "train", "test"]


def plan(n):
    """段数不足 6 时要手工排，否则某些类别可能一个样本都分不到 test"""
    if n >= 6:
        return [PATTERN[i % len(PATTERN)] for i in range(n)]
    return {
        5: ["train", "val", "train", "train", "test"],
        4: ["train", "train", "val", "test"],
        3: ["train", "train", "val"],
        2: ["train", "val"],
    }.get(n, ["train"] * max(n, 1))


def natural(s):
    parts = s.split("_")
    try:
        return int(parts[1])
    except (IndexError, ValueError):
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default=RAW)
    ap.add_argument("--dst", default=DST)
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()

    if not os.path.isdir(args.raw):
        print(f"没有 {args.raw}/，先跑 extract_mark_frames.py")
        return

    segs = [d for d in os.listdir(args.raw) if os.path.isdir(os.path.join(args.raw, d))]

    # 先统计每段的图数，剔掉一张图都没有的段（质量闸门全拒的段会落到这里）
    nimg = {}
    for s in segs:
        nimg[s] = len([f for f in os.listdir(os.path.join(args.raw, s))
                       if f.lower().endswith(".jpg")])
    dead = [s for s, n in nimg.items() if n == 0]
    if dead:
        print(f"跳过空段（质量闸门全拒）：{' '.join(sorted(dead))}")
    segs = [s for s in segs if nimg[s] > 0]

    # 按前缀分组，每组内部按段号排序后套 plan()
    groups = {}
    for s in segs:
        groups.setdefault(s.split("_")[0], []).append(s)

    assign = {k: [] for k in SPLITS}
    stat = []
    for pre in sorted(groups):
        lst = sorted(groups[pre], key=natural)
        for s, sp in zip(lst, plan(len(lst))):
            assign[sp].append((pre, s))
        stat.append((pre, len(lst)))

    print("段分组：", "  ".join(f"{p}={n}" for p, n in stat))

    # 先扫一遍，统计每段的图数、空标注数、类别分布
    detail = {}
    for sp in SPLITS:
        for pre, s in assign[sp]:
            d = os.path.join(args.raw, s)
            imgs = sorted(f for f in os.listdir(d) if f.lower().endswith(".jpg"))
            n_lbl = n_empty = 0
            cls_tally = {0: 0, 1: 0, 2: 0}
            bad = []
            for f in imgs:
                tp = os.path.join(d, os.path.splitext(f)[0] + ".txt")
                if not (os.path.exists(tp) and os.path.getsize(tp) > 0):
                    n_empty += 1
                    continue
                n_lbl += 1
                for line in open(tp, encoding="utf-8"):
                    p = line.split()
                    if len(p) >= 5:
                        c = int(float(p[0]))
                        cls_tally[c] = cls_tally.get(c, 0) + 1
                        want = CLASSES.index(pre) if pre in CLASSES else None
                        if want is not None and c != want:
                            bad.append(f"{f}:{c}")
            detail[(sp, s)] = (imgs, n_lbl, n_empty, cls_tally, bad)

    print(f"\n{'集合':<7}{'段':<9}{'图':>5}{'有框':>5}{'空':>4}{'L':>4}{'T':>4}{'X':>4}")
    print("-" * 46)
    tot = {sp: dict(img=0, lbl=0, cls={0: 0, 1: 0, 2: 0}) for sp in SPLITS}
    for sp in SPLITS:
        for pre, s in assign[sp]:
            imgs, n_lbl, n_empty, tally, _ = detail[(sp, s)]
            print(f"{sp:<7}{s:<9}{len(imgs):5d}{n_lbl:5d}{n_empty:4d}"
                  f"{tally[0]:4d}{tally[1]:4d}{tally[2]:4d}")
            tot[sp]["img"] += len(imgs)
            tot[sp]["lbl"] += n_lbl
            for k in (0, 1, 2):
                tot[sp]["cls"][k] += tally[k]
    print("-" * 46)
    for sp in SPLITS:
        t = tot[sp]
        print(f"{sp:<7}{'合计':<9}{t['img']:5d}{t['lbl']:5d}{t['img'] - t['lbl']:4d}"
              f"{t['cls'][0]:4d}{t['cls'][1]:4d}{t['cls'][2]:4d}")

    allc = {k: sum(tot[sp]["cls"][k] for sp in SPLITS) for k in (0, 1, 2)}
    print(f"\n三类框总数：L={allc[0]}  T={allc[1]}  X={allc[2]}")
    for sp in SPLITS:
        miss = [CLASSES[k] for k in (0, 1, 2) if tot[sp]["cls"][k] == 0]
        if miss:
            print(f"  ⚠️ {sp} 里没有 {'/'.join(miss)} 类样本，该类指标无法评估")

    # 类别写错的提醒
    for (sp, s), (_, _, _, _, bad) in detail.items():
        if bad:
            print(f"  ⚠️ {s} 有 {len(bad)} 个框类别与段名不符（前3个：{bad[:3]}）")

    empty_seg = [s for sp in SPLITS for pre, s in assign[sp]
                 if detail[(sp, s)][1] == 0 and pre != "neg"]
    if empty_seg:
        print(f"  ⚠️ 这些段一张都没标，等于没数据：{' '.join(empty_seg)}")

    if args.dry:
        print("\ndry-run，未复制文件")
        return

    # 复制
    if os.path.isdir(args.dst):
        shutil.rmtree(args.dst)
    for sp in SPLITS:
        os.makedirs(os.path.join(args.dst, "images", sp), exist_ok=True)
        os.makedirs(os.path.join(args.dst, "labels", sp), exist_ok=True)

    n_copy = 0
    for sp in SPLITS:
        for pre, s in assign[sp]:
            imgs, _, _, _, _ = detail[(sp, s)]
            for f in imgs:
                src_img = os.path.join(args.raw, s, f)
                stem = s + "_" + os.path.splitext(f)[0]
                shutil.copy2(src_img, os.path.join(args.dst, "images", sp, stem + ".jpg"))
                src_txt = os.path.join(args.raw, s, os.path.splitext(f)[0] + ".txt")
                dst_txt = os.path.join(args.dst, "labels", sp, stem + ".txt")
                if os.path.exists(src_txt) and os.path.getsize(src_txt) > 0:
                    shutil.copy2(src_txt, dst_txt)
                else:
                    open(dst_txt, "w").close()      # 背景样本：空 txt，训练时当负样本
                n_copy += 1

    with open(os.path.join(args.dst, "data.yaml"), "w", encoding="utf-8") as f:
        f.write("# 由 split_marker_dataset.py 生成\n")
        f.write(f"path: {os.path.abspath(args.dst).replace(os.sep, '/')}\n")
        f.write("train: images/train\nval: images/val\ntest: images/test\n")
        f.write(f"nc: {len(CLASSES)}\nnames: {CLASSES}\n")

    print(f"\n已复制 {n_copy} 张 -> {args.dst}/")
    print("data.yaml:")
    print(open(os.path.join(args.dst, "data.yaml"), encoding="utf-8").read())


if __name__ == "__main__":
    main()
