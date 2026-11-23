# -*- coding: utf-8 -*-
"""
生成一个**极小的合成数据集**，只用来：
  1. 验证 ultralytics 训练链路在本机能否跑通；
  2. 实测"每 epoch 要多久"，从而推算 200~300 张真实数据训练 50 epoch 的耗时。

注意：这是合成图，**绝对不能**用于考核数据集（细则禁止用非实拍数据训练）。
真实数据必须用自己的外接单目相机在实验室采集。

用法：
  python scripts/make_demo_dataset.py --out D:\\vision_ws\\k1_soccer_vision\\_smoke_dataset --n 60
"""

import argparse
import os
import random

import cv2
import numpy as np
import yaml


def synth(i, w=1280, h=720):
    rng = random.Random(i)
    img = np.full((h, w, 3), 0, np.uint8)
    img[:, :] = (35 + rng.randint(0, 25),) * 3
    cv2.rectangle(img, (0, int(h * rng.uniform(0.45, 0.6))), (w, h),
                  (50 + rng.randint(0, 40), 110 + rng.randint(0, 50), 50 + rng.randint(0, 40)), -1)
    # 场地白线
    for x in range(0, w, 160):
        cv2.line(img, (x, int(h * 0.55)), (int(w / 2 + (x - w / 2) * 1.7), h), (235, 235, 235), 3)

    r = rng.randint(14, 55)
    cx = rng.randint(r + 10, w - r - 10)
    cy = rng.randint(int(h * 0.35), h - r - 10)
    cv2.circle(img, (cx, cy), r, (245, 245, 245), -1)
    cv2.circle(img, (cx, cy), r, (25, 25, 25), 2)
    for k in range(3):
        a = rng.uniform(0, np.pi)
        cv2.line(img, (int(cx - r * np.cos(a)), int(cy - r * np.sin(a))),
                 (int(cx + r * np.cos(a)), int(cy + r * np.sin(a))), (120, 120, 120), 1)

    x1, y1, x2, y2 = cx - r, cy - r, cx + r, cy + r
    return img, (x1, y1, x2, y2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--n", type=int, default=60)
    args = ap.parse_args()

    for split, count in (("train", int(args.n * 0.7)), ("val", int(args.n * 0.2)), ("test", args.n)):
        for sub in ("images", "labels"):
            os.makedirs(os.path.join(args.out, sub, split), exist_ok=True)

    splits = {"train": int(args.n * 0.7), "val": int(args.n * 0.2), "test": int(args.n * 0.1)}
    i = 0
    for split, count in splits.items():
        for _ in range(count):
            img, (x1, y1, x2, y2) = synth(i)
            h, w = img.shape[:2]
            name = f"synth_{i:05d}"
            cv2.imwrite(os.path.join(args.out, "images", split, name + ".jpg"), img)
            cx = (x1 + x2) / 2 / w
            cy = (y1 + y2) / 2 / h
            bw = (x2 - x1) / w
            bh = (y2 - y1) / h
            with open(os.path.join(args.out, "labels", split, name + ".txt"), "w") as f:
                f.write(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
            i += 1

    data = {
        "path": os.path.abspath(args.out),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "nc": 1,
        "names": ["ball"],
    }
    yaml_path = os.path.join(args.out, "data.yaml")
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print("dataset ->", args.out)
    print("data.yaml ->", yaml_path)
    print("split sizes:", splits)


if __name__ == "__main__":
    main()
