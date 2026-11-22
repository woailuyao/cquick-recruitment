#!/usr/bin/env python3
"""在测试集上自动找出漏检 / 误检样本，供 README 的失败案例分析。

漏检 = 标注有球，但模型没检出（或 IoU 太低）
误检 = 模型检出框，但与所有标注框 IoU 都很低（把别的东西认成球）

用法：
  python tools\\find_failures.py --model models\\ball_best.pt --data soccer_dataset
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2                          # noqa: E402
import numpy as np                  # noqa: E402


def iou(a, b) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, x2 - x1), max(0.0, y2 - y1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def load_gt(lbl_path, w, h):
    boxes = []
    if not os.path.exists(lbl_path):
        return boxes
    for ln in open(lbl_path, encoding="utf-8"):
        t = ln.split()
        if len(t) < 5:
            continue
        cx, cy, bw, bh = map(float, t[1:5])
        boxes.append(((cx - bw / 2) * w, (cy - bh / 2) * h,
                      (cx + bw / 2) * w, (cy + bh / 2) * h))
    return boxes


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.path.join(ROOT, "models", "ball_best.pt"))
    ap.add_argument("--data", default=os.path.join(ROOT, "soccer_dataset"))
    ap.add_argument("--split", default="test")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou-thr", type=float, default=0.3)
    ap.add_argument("--save-dir", default=os.path.join(ROOT, "logs", "failures"))
    args = ap.parse_args()

    from ultralytics import YOLO

    img_dir = os.path.join(args.data, "images", args.split)
    lbl_dir = os.path.join(args.data, "labels", args.split)
    imgs = sorted(glob.glob(os.path.join(img_dir, "*.jpg")) +
                  glob.glob(os.path.join(img_dir, "*.png")))
    if not imgs:
        print(f"[error] {img_dir} 里没有图片")
        return 2
    os.makedirs(args.save_dir, exist_ok=True)
    model = YOLO(args.model)

    missed, false_pos = [], []
    n_gt = n_det = 0

    for p in imgs:
        im = cv2.imread(p)
        h, w = im.shape[:2]
        stem = os.path.splitext(os.path.basename(p))[0]
        gt = load_gt(os.path.join(lbl_dir, stem + ".txt"), w, h)
        r = model(im, conf=args.conf, verbose=False)[0]
        det = [tuple(float(v) for v in b) for b in
               (r.boxes.xyxy.cpu().numpy() if r.boxes is not None and len(r.boxes) else [])]
        n_gt += len(gt)
        n_det += len(det)

        # 漏检：标注框没有匹配到检测框
        for g in gt:
            best = max((iou(g, d) for d in det), default=0.0)
            if best < args.iou_thr:
                missed.append((p, g, len(det), best))

        # 误检：检测框没有匹配到任何标注
        for d in det:
            best = max((iou(g, d) for g in gt), default=0.0)
            if best < args.iou_thr:
                false_pos.append((p, d, len(gt), best))

    def dump(items, tag, box_idx):
        print(f"\n=== {tag}（{len(items)} 个）===")
        for i, (p, box, other, best) in enumerate(items[:8], 1):
            im = cv2.imread(p)
            cv2.rectangle(im, (int(box[0]), int(box[1])),
                          (int(box[2]), int(box[3])), (0, 0, 255), 3)
            label = f"{tag}{i}"
            cv2.putText(im, label, (int(box[0]) + 4, max(24, int(box[1]) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            out = os.path.join(args.save_dir, f"{tag}_{i}_{os.path.basename(p)}")
            cv2.imwrite(out, im)
            size = f"{box[2]-box[0]:.0f}x{box[3]-box[1]:.0f}px"
            print(f"  {i}. {os.path.basename(p):28s} 框 {size:12s} "
                  f"该图标注 {other if tag=='漏检' else other} 个  最佳IoU {best:.2f}")
            print(f"     已存标注图 -> {out}")

    print(f"[统计] {args.split} 集：{len(imgs)} 张图，标注 {n_gt} 个球，"
          f"模型检出 {n_det} 个框（conf≥{args.conf}）")
    dump(missed, "漏检", 0)
    dump(false_pos, "误检", 1)
    print(f"\n对比图存在 {args.save_dir}，打开看看，挑一张漏检一张误检写进 README")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
