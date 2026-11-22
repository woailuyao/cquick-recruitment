#!/usr/bin/env python3
"""标注质量检查（考核 1.5）：画回 bbox、越界/宽高检查、泄漏检查。

用法：
    python check_labels.py --data soccer_dataset --sample 30
"""

import argparse
import os
import random
from pathlib import Path

import cv2
import numpy as np


def parse_label(line, img_w, img_h):
    """解析一行 YOLO 标签，返回 (class_id, cx, cy, w, h) 或 None。"""
    parts = line.strip().split()
    if len(parts) != 5:
        return None
    cid = int(float(parts[0]))
    cx, cy, w, h = [float(x) for x in parts[1:]]
    # 越界检查
    if not (0 <= cx <= 1 and 0 <= cy <= 1 and w > 0 and h > 0):
        return None
    return cid, cx, cy, w, h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="soccer_dataset")
    ap.add_argument("--sample", type=int, default=30)
    args = ap.parse_args()

    images_dir = os.path.join(args.data, "images")
    labels_dir = os.path.join(args.data, "labels")

    all_imgs = []
    for split in ["train", "val", "test"]:
        d = os.path.join(images_dir, split)
        if os.path.isdir(d):
            for f in sorted(os.listdir(d)):
                if f.lower().endswith((".jpg", ".png")):
                    all_imgs.append((split, f))

    print(f"共发现 {len(all_imgs)} 张图片")

    # 随机抽查
    sample = random.sample(all_imgs, min(args.sample, len(all_imgs)))
    issues = 0
    for split, fname in sample:
        img_path = os.path.join(images_dir, split, fname)
        lbl_path = os.path.join(labels_dir, split, Path(fname).with_suffix(".txt").name)
        img = cv2.imread(img_path)
        if img is None:
            print(f"[FAIL] 无法读取 {img_path}")
            issues += 1
            continue
        h, w = img.shape[:2]
        if not os.path.exists(lbl_path):
            print(f"[WARN] {fname}: 缺少标签文件")
            continue
        with open(lbl_path) as f:
            for line in f:
                r = parse_label(line, w, h)
                if r is None:
                    print(f"[FAIL] {fname}: 标签格式错误/越界: {line.strip()}")
                    issues += 1
                    continue
                cid, cx, cy, bw, bh = r
                x1 = int((cx - bw / 2) * w)
                y1 = int((cy - bh / 2) * h)
                x2 = int((cx + bw / 2) * w)
                y2 = int((cy + bh / 2) * h)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # 可选：显示（无 GUI 环境请设 SHOW=0）
        if os.environ.get("SHOW", "0") == "1":
            cv2.imshow("check", img)
            cv2.waitKey(0)

    # 泄漏检查：同一视频片段的相邻帧是否跨 split
    print("\n泄漏检查：检查 train/val/test 是否混入同一视频片段")
    stem_sets = {}
    for split in ["train", "val", "test"]:
        d = os.path.join(images_dir, split)
        if os.path.isdir(d):
            stems = {Path(f).stem.rsplit("_", 1)[0] for f in os.listdir(d)}
            stem_sets[split] = stems
    overlap = False
    for a in ["train", "val", "test"]:
        for b in ["train", "val", "test"]:
            if a < b and a in stem_sets and b in stem_sets:
                inter = stem_sets[a] & stem_sets[b]
                if inter:
                    overlap = True
                    print(f"[FAIL] {a} 与 {b} 共享视频片段: {inter}")
    if not overlap:
        print("[OK] 无跨集合视频片段泄漏")

    print(f"\n检查完成，发现 {issues} 个问题")


if __name__ == "__main__":
    main()
