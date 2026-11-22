# -*- coding: utf-8 -*-
"""统计检测器在 test 集上的"框中心"像素误差。

mAP 衡量框贴得准不准，但单目定位真正用的是框中心坐标，
所以这里单独统计中心点误差，和 mAP 是两套指标。

用法:
    python eval_center_error.py
    python eval_center_error.py --model weights/marker_v3_best.pt --imgsz 960 --out docs/center_error.csv
"""

import argparse
import csv
import os
from collections import defaultdict

import numpy as np

# 当前 test 集由 marker_dataset2 / marker_dataset3 的 test 划分组成
IMG_DIRS = [
    "D:/vision_data/marker_dataset2/images/test",
    "D:/vision_data/marker_dataset3/images/test",
]
LBL_DIRS = [
    "D:/vision_data/marker_dataset2/labels/test",
    "D:/vision_data/marker_dataset3/labels/test",
]
DEFAULT_MODEL = "weights/marker_v3_best.pt"


def load_gt(label_path, w, h):
    """返回 [(cls, x1, y1, x2, y2), ...]，坐标为原图像素。"""
    out = []
    if not os.path.exists(label_path):
        return out
    with open(label_path, "r", encoding="utf-8") as f:
        for line in f:
            p = line.split()
            if len(p) < 5:
                continue
            c = int(float(p[0]))
            x, y, bw, bh = map(float, p[1:5])
            out.append((
                c,
                (x - bw / 2) * w, (y - bh / 2) * h,
                (x + bw / 2) * w, (y + bh / 2) * h,
            ))
    return out


def iou_one(a, b):
    x1 = max(a[0], b[0]); y1 = max(a[1], b[1])
    x2 = min(a[2], b[2]); y2 = min(a[3], b[3])
    iw = max(0.0, x2 - x1); ih = max(0.0, y2 - y1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def center(b):
    return ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--conf", type=float, default=0.25, help="检测置信度阈值")
    ap.add_argument("--iou-match", type=float, default=0.30, help="GT 与预测的匹配 IoU 下限")
    ap.add_argument("--images", default=",".join(IMG_DIRS),
                    help="测试图片目录，多个目录用逗号分隔")
    ap.add_argument("--labels", default=",".join(LBL_DIRS),
                    help="对应标签目录，多个目录用逗号分隔")
    ap.add_argument("--out", default="docs/center_error.csv")
    args = ap.parse_args()

    from ultralytics import YOLO

    if not os.path.exists(args.model):
        raise SystemExit(f"找不到模型: {args.model}")

    img_dirs = [d.strip() for d in args.images.split(",") if d.strip()]
    lbl_dirs = [d.strip() for d in args.labels.split(",") if d.strip()]
    if len(img_dirs) != len(lbl_dirs):
        raise SystemExit("--images 与 --labels 的目录数量不一致")

    pairs = []
    for img_dir, lbl_dir in zip(img_dirs, lbl_dirs):
        if not os.path.isdir(img_dir):
            raise SystemExit(f"目录不存在: {img_dir}")
        for fn in sorted(os.listdir(img_dir)):
            if fn.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                pairs.append((
                    fn,
                    os.path.join(img_dir, fn),
                    os.path.join(lbl_dir, os.path.splitext(fn)[0] + ".txt"),
                ))
    if not pairs:
        raise SystemExit(f"目录里没有图片: {', '.join(img_dirs)}")

    model = YOLO(args.model)
    names = model.names

    per_cls = defaultdict(lambda: {"err": [], "gt": 0, "hit": 0, "miss": 0, "fp": 0})
    rows = []

    for i, (fn, img_path, lbl_path) in enumerate(pairs, 1):
        res = model.predict(
            source=img_path,
            imgsz=args.imgsz,
            conf=args.conf,
            iou=0.7,
            verbose=False,
            device=0,
        )[0]

        h, w = res.orig_shape[:2]
        gt = load_gt(lbl_path, w, h)

        if res.boxes is not None and len(res.boxes) > 0:
            pb = res.boxes.xyxy.cpu().numpy()
            pc = res.boxes.cls.cpu().numpy().astype(int)
            pconf = res.boxes.conf.cpu().numpy()
        else:
            pb = np.zeros((0, 4)); pc = np.zeros(0, dtype=int); pconf = np.zeros(0)

        used = np.zeros(len(pb), dtype=bool)

        for (c, *gt_box) in gt:
            gt_box = list(gt_box)
            st = per_cls[c]
            st["gt"] += 1

            best_i, best_v = -1, 0.0
            for j in range(len(pb)):
                if used[j] or pc[j] != c:
                    continue
                v = iou_one(gt_box, pb[j])
                if v > best_v:
                    best_i, best_v = j, v

            if best_i >= 0 and best_v >= args.iou_match:
                used[best_i] = True
                cx1, cy1 = center(gt_box)
                cx2, cy2 = center(pb[best_i])
                d = float(np.hypot(cx1 - cx2, cy1 - cy2))
                st["hit"] += 1
                st["err"].append(d)
                rows.append([fn, names.get(c, c), round(d, 2), round(float(pconf[best_i]), 3),
                             round(best_v, 3)])
            else:
                st["miss"] += 1
                rows.append([fn, names.get(c, c), "MISS", "", ""])

        for j in range(len(pb)):
            if not used[j]:
                per_cls[int(pc[j])]["fp"] += 1

        if i % 20 == 0:
            print(f"  ... {i}/{len(pairs)}")

    # ---------- 汇总 ----------
    print()
    print(f"模型: {args.model}   imgsz={args.imgsz}   conf>={args.conf}   match_IoU>={args.iou_match}")
    print(f"{'类别':<10}{'GT':>5}{'命中':>6}{'漏检':>6}{'误检':>6}"
          f"{'中位误差':>10}{'均值':>8}{'P90':>8}{'<=5px':>8}")
    print("-" * 72)

    overall = {"err": [], "gt": 0, "hit": 0, "miss": 0, "fp": 0}
    for c in sorted(per_cls):
        st = per_cls[c]
        for k in ("gt", "hit", "miss", "fp"):
            overall[k] += st[k]
        overall["err"].extend(st["err"])

        if st["err"]:
            a = np.array(st["err"])
            med, mean, p90 = np.median(a), a.mean(), np.percentile(a, 90)
            frac5 = 100.0 * (a <= 5).mean()
            print(f"{str(names.get(c, c)):<10}{st['gt']:>5}{st['hit']:>6}{st['miss']:>6}{st['fp']:>6}"
                  f"{med:>10.2f}{mean:>8.2f}{p90:>8.2f}{frac5:>7.1f}%")
        else:
            print(f"{str(names.get(c, c)):<10}{st['gt']:>5}{st['hit']:>6}{st['miss']:>6}{st['fp']:>6}"
                  f"{'-':>10}{'-':>8}{'-':>8}{'-':>8}")

    if overall["err"]:
        a = np.array(overall["err"])
        print("-" * 72)
        print(f"{'合计':<10}{overall['gt']:>5}{overall['hit']:>6}{overall['miss']:>6}{overall['fp']:>6}"
              f"{np.median(a):>10.2f}{a.mean():>8.2f}{np.percentile(a, 90):>8.2f}"
              f"{100.0 * (a <= 5).mean():>7.1f}%")

    with open(args.out, "w", newline="", encoding="utf-8-sig") as f:
        w_ = csv.writer(f)
        w_.writerow(["图片", "类别", "中心误差px", "置信度", "匹配IoU"])
        w_.writerows(rows)
    print(f"\n逐框明细已写入: {args.out}")

    if overall["err"]:
        worst = sorted(
            [r for r in rows if isinstance(r[2], float)],
            key=lambda r: -r[2],
        )[:5]
        print("误差最大的 5 个框:")
        for r in worst:
            print(f"  {r[0]}  {r[1]}  误差 {r[2]} px  conf {r[3]}")


if __name__ == "__main__":
    main()