#!/usr/bin/env python3
"""PyTorch vs ONNX 推理对比（加分项 +3）。

对应细则加分项："导出 ONNX，并在同一测试片段上完成 PyTorch / ONNX 推理对比"。

用法：
  python tools\\bench_onnx.py --pt models\\ball_best.pt --images soccer_dataset\\images\\test --n 30
"""
from __future__ import annotations

import argparse
import glob
import os
import statistics
import sys
import time

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


def pick_images(folder, n):
    fs = sorted(glob.glob(os.path.join(folder, "*.jpg")) +
                glob.glob(os.path.join(folder, "*.png")))
    if not fs:
        raise SystemExit(f"[error] {folder} 里没有图片")
    step = max(1, len(fs) // n)
    return fs[::step][:n]


def run_model(model, imgs, conf=0.25):
    """返回 (每张耗时列表, 每张检测框列表)。"""
    times = []
    dets = []
    for p in imgs:
        im = cv2.imread(p)
        t0 = time.perf_counter()
        r = model(im, conf=conf, verbose=False)
        times.append((time.perf_counter() - t0) * 1000.0)
        boxes = []
        if r and r[0].boxes is not None and len(r[0].boxes):
            for b in r[0].boxes.xyxy.cpu().numpy():
                boxes.append(tuple(float(v) for v in b))
        dets.append(boxes)
    return times, dets


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pt", default=os.path.join(ROOT, "models", "ball_best.pt"))
    ap.add_argument("--images", default=os.path.join(ROOT, "soccer_dataset", "images", "test"))
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--warmup", type=int, default=3)
    args = ap.parse_args()

    from ultralytics import YOLO

    if not os.path.exists(args.pt):
        print(f"[error] 找不到权重 {args.pt}")
        return 2
    imgs = pick_images(args.images, args.n)
    print(f"[bench] 权重 {os.path.basename(args.pt)}   测试图 {len(imgs)} 张   "
          f"imgsz={args.imgsz}")

    pt = YOLO(args.pt)
    for _ in range(args.warmup):                       # 预热
        pt(cv2.imread(imgs[0]), conf=args.conf, imgsz=args.imgsz, verbose=False)
    t_pt, d_pt = run_model(pt, imgs, args.conf)

    # 导出 ONNX
    onnx_path = os.path.splitext(args.pt)[0] + ".onnx"
    if not os.path.exists(onnx_path):
        print("[bench] 导出 ONNX ...")
        out = pt.export(format="onnx", imgsz=args.imgsz, simplify=True)
        onnx_path = str(out)
    print(f"[bench] ONNX: {os.path.basename(onnx_path)} "
          f"({os.path.getsize(onnx_path)/1e6:.1f} MB)")

    onx = YOLO(onnx_path, task="detect")
    for _ in range(args.warmup):
        onx(cv2.imread(imgs[0]), conf=args.conf, imgsz=args.imgsz, verbose=False)
    t_onx, d_onx = run_model(onx, imgs, args.conf)

    def stat(t):
        return (statistics.mean(t), statistics.median(t), min(t))

    m_pt, med_pt, min_pt = stat(t_pt)
    m_onx, med_onx, min_onx = stat(t_onx)

    # 检测结果一致性
    total_pt = sum(len(d) for d in d_pt)
    total_onx = sum(len(d) for d in d_onx)
    ious = []
    for a, b in zip(d_pt, d_onx):
        for box in a:
            if b:
                ious.append(max(iou(box, bb) for bb in b))
    mean_iou = float(np.mean(ious)) if ious else 0.0

    print("\n=== 对比结果（同一批测试图，CPU）===")
    print(f"{'推理后端':<12}{'平均耗时':>10}{'中位耗时':>10}{'最快':>9}{'等效FPS':>10}{'检出框':>8}")
    print(f"{'PyTorch':<12}{m_pt:>8.1f}ms{med_pt:>8.1f}ms{min_pt:>7.1f}ms"
          f"{1000.0/m_pt:>9.1f}{total_pt:>8}")
    print(f"{'ONNX':<12}{m_onx:>8.1f}ms{med_onx:>8.1f}ms{min_onx:>7.1f}ms"
          f"{1000.0/m_onx:>9.1f}{total_onx:>8}")
    speedup = m_pt / m_onx if m_onx > 0 else float("inf")
    print(f"\n加速比：ONNX 比 PyTorch 快 {speedup:.2f}x")
    print(f"检测一致性：PyTorch {total_pt} 个框，ONNX {total_onx} 个框，"
          f"匹配框平均 IoU {mean_iou:.4f}")
    if mean_iou > 0.95 and abs(total_pt - total_onx) <= max(2, 0.05 * total_pt):
        print("[ok] 两者检测结果一致，ONNX 可用于部署")
    else:
        print("[warn] 两者结果有差异，检查导出参数（imgsz / opset / 是否 dynamic）")

    print("\n（把上面这张表抄进 README 的加分项说明）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
