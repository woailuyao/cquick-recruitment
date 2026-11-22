# -*- coding: utf-8 -*-
"""抽查足球标注质量：随机抽 N 张训练图，把 YOLO 标签框画回原图并拼成一张图。

用法:
    python check_ball_labels.py --dataset D:/vision_data/ball_dataset --n 30 --out ../docs/ball_label_check.jpg
"""

import argparse
import os
import random

import cv2
import numpy as np


def load_labels(txt_path, w, h):
    boxes = []
    if not os.path.exists(txt_path):
        return boxes
    with open(txt_path, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.split()
            if len(parts) < 5:
                continue
            c, cx, cy, bw, bh = parts[:5]
            cx, cy, bw, bh = float(cx) * w, float(cy) * h, float(bw) * w, float(bh) * h
            boxes.append((int(c), int(cx - bw / 2), int(cy - bh / 2),
                          int(cx + bw / 2), int(cy + bh / 2)))
    return boxes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", default="D:/vision_data/ball_dataset")
    ap.add_argument("--split", default="train")
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--tile", type=int, default=320)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="../docs/ball_label_check.jpg")
    args = ap.parse_args()

    img_dir = os.path.join(args.dataset, "images", args.split)
    lbl_dir = os.path.join(args.dataset, "labels", args.split)
    names = sorted(os.listdir(img_dir))
    random.Random(args.seed).shuffle(names)
    names = names[:args.n]

    tiles = []
    for name in names:
        path = os.path.join(img_dir, name)
        img = cv2.imread(path)
        if img is None:
            continue
        h, w = img.shape[:2]
        boxes = load_labels(os.path.join(lbl_dir, os.path.splitext(name)[0] + ".txt"), w, h)
        for _, x1, y1, x2, y2 in boxes:
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
        scale = args.tile / float(w)
        img = cv2.resize(img, (args.tile, int(h * scale)))
        cv2.putText(img, name, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        tiles.append(img)

    if not tiles:
        raise SystemExit("没有找到可用图片")

    th = tiles[0].shape[0]
    rows = []
    for i in range(0, len(tiles), args.cols):
        row = tiles[i:i + args.cols]
        while len(row) < args.cols:
            row.append(np.zeros((th, args.tile, 3), np.uint8))
        rows.append(np.hstack(row))
    grid = np.vstack(rows)

    out = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), args.out))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    # cv2.imwrite 在 Windows 下不支持含中文的路径，改用 imencode + 二进制写入
    ok, buf = cv2.imencode(".jpg", grid, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        raise SystemExit("编码失败")
    with open(out, "wb") as f:
        f.write(buf.tobytes())
    print(f"已保存 {out}  （{len(tiles)} 张，{grid.shape[1]}x{grid.shape[0]}）")


if __name__ == "__main__":
    main()