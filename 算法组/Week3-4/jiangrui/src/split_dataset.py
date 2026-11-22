# -*- coding: utf-8 -*-
"""把采集到的原图按“连续帧块”划分到 train / val / test。

为什么不随机逐帧划分：
    采集时是边走边抽帧，相邻帧几乎是同一画面。如果随机分，验证集里会混进
    训练集的相邻帧，mAP 会被抬高（上一版就是这样虚高的）。
    所以先按连续 5 帧切块，整块只进一个集合，保证三个集合之间没有近重复。

用法：
    python split_dataset.py
    python split_dataset.py --src ...\\images\\raw --ratio 48,10,10
"""

import argparse
import glob
import os
import random
import shutil


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=r"D:\vision_data\marker_dataset2")
    ap.add_argument("--src", default=None, help="默认 <root>\\images\\raw")
    ap.add_argument("--ratio", default="48,10,10", help="train,val,test 张数")
    ap.add_argument("--chunk", type=int, default=5, help="连续帧块大小")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--copy", action="store_true", default=True)
    return ap.parse_args()


def main():
    args = parse_args()
    root = os.path.abspath(args.root)
    src = os.path.abspath(args.src or os.path.join(root, "images", "raw"))
    files = sorted(glob.glob(os.path.join(src, "*.jpg")) +
                   glob.glob(os.path.join(src, "*.png")))
    if not files:
        raise SystemExit(f"{src} 里没有图片")

    n_train, n_val, n_test = [int(x) for x in args.ratio.split(",")]
    if n_train + n_val + n_test != len(files):
        raise SystemExit(f"分配数 {n_train}+{n_val}+{n_test} 与图片数 {len(files)} 不符，"
                         f"改成 --ratio {len(files)},{0},{0} 这样的格式重试")

    chunks = [files[i:i + args.chunk] for i in range(0, len(files), args.chunk)]
    random.Random(args.seed).shuffle(chunks)

    plan = {"train": [], "val": [], "test": []}
    quota = {"train": n_train, "val": n_val, "test": n_test}
    for ch in chunks:
        for split in ("train", "val", "test"):
            if quota[split] >= len(ch):
                plan[split].extend(ch)
                quota[split] -= len(ch)
                break
        else:
            # 剩余名额不够整块，塞给还有空位的集合（通常不会发生）
            for split in ("test", "val", "train"):
                if quota[split] > 0:
                    take = ch[:quota[split]]
                    plan[split].extend(take)
                    quota[split] -= len(take)
                    break

    for split, items in plan.items():
        img_dir = os.path.join(root, "images", split)
        lbl_dir = os.path.join(root, "labels", split)
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(lbl_dir, exist_ok=True)
        with open(os.path.join(lbl_dir, "classes.txt"), "w",
                  encoding="utf-8", newline="\n") as f:
            f.write("TCross\nLCross\n")
        for p in items:
            dst = os.path.join(img_dir, os.path.basename(p))
            if not os.path.exists(dst):
                shutil.copy2(p, dst)
        print(f"{split:<6} {len(items):>3} 张 -> {img_dir}")

    # 有的话把已有标注一起带过去
    src_lbl = os.path.join(root, "labels", "raw")
    if os.path.isdir(src_lbl):
        for split, items in plan.items():
            for p in items:
                t = os.path.join(src_lbl, os.path.splitext(os.path.basename(p))[0] + ".txt")
                if os.path.exists(t):
                    shutil.copy2(t, os.path.join(root, "labels", split,
                                                 os.path.basename(t)))

    yml = os.path.join(root, "marker_data.yaml")
    with open(yml, "w", encoding="utf-8", newline="\n") as f:
        f.write(f"""# 补拍数据：同机位、多距离
path: {root.replace(os.sep, "/")}
train: images/train
val: images/val
test: images/test

names:
  0: TCross
  1: LCross
""")
    print("已写入", yml)

    man = os.path.join(root, "split.txt")
    with open(man, "w", encoding="utf-8", newline="\n") as f:
        for split, items in plan.items():
            for p in items:
                f.write(f"{split}\t{os.path.basename(p)}\n")
    print("划分清单", man)


if __name__ == "__main__":
    main()