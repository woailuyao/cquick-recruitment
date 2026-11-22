#!/usr/bin/env python3

import argparse
import glob
import os
import shutil

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRAMES = os.path.join(ROOT, "data", "frames")

CLASSES = {
    "ball": ["ball"],
    "marker": ["LCross", "TCross", "XCross", "PenaltyPoint"],
}

def frame_score(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", choices=sorted(CLASSES), required=True)
    ap.add_argument("--videos", nargs="+", type=int, required=True)
    ap.add_argument("--val", nargs="+", type=int, default=[])
    ap.add_argument("--min-sharp", type=float, default=0.0,
                    help="清晰度下限，0 表示按分位数自动")
    ap.add_argument("--dedup", type=float, default=2.0,
                    help="与上一张保留帧的平均灰度差小于此值则视为重复")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    out_root = args.out or os.path.join(ROOT, "dataset", args.task)
    images_train = os.path.join(out_root, "images", "train")
    images_val = os.path.join(out_root, "images", "val")
    labels_train = os.path.join(out_root, "labels", "train")
    labels_val = os.path.join(out_root, "labels", "val")
    for d in (images_train, images_val, labels_train, labels_val):
        os.makedirs(d, exist_ok=True)

    all_frames = []
    for v in args.videos:
        files = sorted(glob.glob(os.path.join(FRAMES, f"video{v}", "*.jpg")))
        for f in files:
            all_frames.append((v, f))

    scored = []
    for v, f in all_frames:
        img = cv2.imread(f)
        if img is None:
            continue
        scored.append((frame_score(img), v, f))

    sharp_vals = np.array([s for s, _, _ in scored])
    threshold = args.min_sharp if args.min_sharp > 0 else float(np.percentile(sharp_vals, 40))
    print(f"总帧 {len(scored)}，清晰度阈值 {threshold:.1f}（中位数 {np.median(sharp_vals):.1f}）")

    kept = []
    prev_small = None
    for s, v, f in scored:
        if s < threshold:
            continue
        img = cv2.imread(f)
        small = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (160, 90))
        if prev_small is not None and float(np.mean(cv2.absdiff(small, prev_small))) < args.dedup:
            continue
        prev_small = small
        kept.append((v, f))

    print(f"筛选后保留 {len(kept)} 张")

    val_set = set(args.val)
    counts = {"train": 0, "val": 0}
    for v, f in kept:
        split = "val" if v in val_set else "train"
        dst_img = os.path.join(images_train if split == "train" else images_val,
                               f"video{v}_{os.path.basename(f)}")
        shutil.copy(f, dst_img)
        label = os.path.join(labels_train if split == "train" else labels_val,
                             os.path.splitext(os.path.basename(dst_img))[0] + ".txt")
        open(label, "w").close()
        counts[split] += 1

    data = {
        "path": out_root.replace("\\", "/"),
        "train": "images/train",
        "val": "images/val",
        "names": {i: n for i, n in enumerate(CLASSES[args.task])},
    }
    with open(os.path.join(out_root, "data.yaml"), "w", encoding="utf-8") as fp:
        yaml.safe_dump(data, fp, allow_unicode=True, sort_keys=False)

    print(f"train={counts['train']}  val={counts['val']}")
    print("data.yaml 已写入", os.path.join(out_root, "data.yaml"))

if __name__ == "__main__":
    main()
