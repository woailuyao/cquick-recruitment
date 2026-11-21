#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动找出误检 / 漏检，并把出错区域裁出来放大。"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


def iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def center_dist(a, b) -> float:
    acx, acy = (a[0] + a[2]) / 2, (a[1] + a[3]) / 2
    bcx, bcy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
    return float(np.hypot(acx - bcx, acy - bcy))


def load_gt(txt: Path, w: int, h: int):
    boxes = []
    if txt.exists():
        for line in txt.read_text(encoding="utf-8").splitlines():
            p = line.split()
            if len(p) < 5:
                continue
            cx, cy, bw, bh = map(float, p[1:5])
            boxes.append((int((cx - bw / 2) * w), int((cy - bh / 2) * h),
                          int((cx + bw / 2) * w), int((cy + bh / 2) * h)))
    return boxes


def crop_around(img, box, pad=140, size=320):
    x1, y1, x2, y2 = box
    cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
    h, w = img.shape[:2]
    x0 = max(0, cx - size // 2)
    y0 = max(0, cy - size // 2)
    x1c = min(w, x0 + size)
    y1c = min(h, y0 + size)
    patch = img[y0:y1c, x0:x1c].copy()
    patch = cv2.resize(patch, (size * 2, size * 2), interpolation=cv2.INTER_NEAREST)
    k = (size * 2) / max(1, (x1c - x0))
    return patch, (x0, y0, k)


def main() -> int:
    ap = argparse.ArgumentParser(description="找误检/漏检")
    ap.add_argument("--model", default="runs/detect/ball_640/weights/best.pt")
    ap.add_argument("--split", default="test")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.3, help="匹配用的 IoU 阈值")
    ap.add_argument("--top", type=int, default=10, help="最多列出多少个")
    ap.add_argument("--out", default="runs/eval/errors")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    img_dir = Path("dataset/images") / args.split
    lab_dir = Path("dataset/labels") / args.split
    images = sorted(img_dir.glob("*.jpg"))
    if not images:
        print(f"没有图片：{img_dir}")
        return 1

    model = YOLO(args.model)
    rows, misses, fps = [], [], []

    for img_path in images:
        data = np.fromfile(str(img_path), dtype=np.uint8)
        img = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if img is None:
            continue
        h, w = img.shape[:2]
        gt = load_gt(lab_dir / (img_path.stem + ".txt"), w, h)
        res = model.predict(img, imgsz=args.imgsz, conf=args.conf,
                            verbose=False, device="cpu")[0]
        preds = [[int(v) for v in b.xyxy[0]] for b in res.boxes]
        confs = [float(b.conf[0]) for b in res.boxes]

        used = set()
        n_miss = 0
        for g in gt:
            best, bi = 0.0, -1
            for i, p in enumerate(preds):
                if i in used:
                    continue
                v = iou(g, p)
                if v > best:
                    best, bi = v, i
            if best >= args.iou:
                used.add(bi)
            else:
                n_miss += 1
                if len(misses) < args.top * 2:
                    patch, (x0, y0, k) = crop_around(img, g)
                    cv2.rectangle(patch, (int((g[0] - x0) * k), int((g[1] - y0) * k)),
                                  (int((g[2] - x0) * k), int((g[3] - y0) * k)), (0, 255, 255), 2)
                    cv2.putText(patch, f"MISS {img_path.stem}", (6, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    ok, buf = cv2.imencode(".jpg", patch)
                    if ok:
                        buf.tofile(str(out / f"miss_{img_path.stem}.jpg"))
                    misses.append((img_path.name, g))

        n_fp = 0
        for i, p in enumerate(preds):
            if i in used:
                continue
            if max((iou(p, g) for g in gt), default=0.0) >= args.iou:
                continue
            n_fp += 1
            if len(fps) < args.top * 2:
                patch, (x0, y0, k) = crop_around(img, p)
                cv2.rectangle(patch, (int((p[0] - x0) * k), int((p[1] - y0) * k)),
                              (int((p[2] - x0) * k), int((p[3] - y0) * k)), (0, 255, 0), 2)
                cv2.putText(patch, f"FP {img_path.stem} {confs[i]:.2f}", (6, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                ok, buf = cv2.imencode(".jpg", patch)
                if ok:
                    buf.tofile(str(out / f"fp_{img_path.stem}.jpg"))
                fps.append((img_path.name, p, confs[i]))

        rows.append({"image": img_path.name, "gt": len(gt), "pred": len(preds),
                     "miss": n_miss, "fp": n_fp})

    with (out.parent / "errors.csv").open("w", newline="", encoding="utf-8-sig") as f:
        wr = csv.DictWriter(f, fieldnames=["image", "gt", "pred", "miss", "fp"])
        wr.writeheader()
        wr.writerows(rows)

    total_gt = sum(r["gt"] for r in rows)
    total_miss = sum(r["miss"] for r in rows)
    total_fp = sum(r["fp"] for r in rows)
    print(f"共 {len(rows)} 张，真值 {total_gt} 个，漏检 {total_miss} 个，误检 {total_fp} 个")
    print(f"（conf={args.conf}, IoU 匹配阈值={args.iou}）\n")

    print("=== 漏检最多的图 ===")
    for r in sorted(rows, key=lambda x: -x["miss"])[:args.top]:
        if r["miss"]:
            print(f"  {r['image']:24s} 真值 {r['gt']}, 预测 {r['pred']}, 漏 {r['miss']}")
    print("\n=== 误检最多的图 ===")
    for r in sorted(rows, key=lambda x: -x["fp"])[:args.top]:
        if r["fp"]:
            print(f"  {r['image']:24s} 真值 {r['gt']}, 预测 {r['pred']}, 误 {r['fp']}")

    print(f"\n裁剪图在：{out.resolve()}")
    print("  miss_*.jpg = 漏检（黄框是球的位置）")
    print("  fp_*.jpg   = 误检（绿框是模型错误框住的东西）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
