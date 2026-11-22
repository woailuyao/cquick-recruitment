# -*- coding: utf-8 -*-
"""train_landmarks.py —— 场地标志（L / T 两类）YOLOv8 训练脚本

在 WSL 的训练环境里跑（ultralytics + torch cu130）：
  ~/vision_ws/k1_soccer_vision/.venv/bin/python train_landmarks.py

做的事：
  1. 读 data.yaml（由 prepare_landmark_dataset.py 生成）
  2. YOLOv8n 训练，自动挑 GPU、开 RAM 缓存（数据集小，缓存后 epoch 很快）
  3. 训完用 val 集复算指标，按类别打印 mAP50 / mAP50-95
  4. 把 best.pt 复制到固定路径（避免 runs/ 目录多了以后找不到）
  5. 写一份 train_summary.md，记录超参与指标，方便写考核报告

典型用法：
  # 跑通链路 / 基线（伪标注也行）
  ... python train_landmarks.py --epochs 40 --name baseline_pseudo
  # 标注完成后正式训练
  ... python train_landmarks.py --epochs 120 --name v1
"""
import argparse
import json
import os
import shutil
import time

import numpy as np

from ultralytics import YOLO


def per_class_metrics(m):
    """从 ultralytics 的 metrics 对象里取每个类别的 mAP50 / mAP50-95。"""
    names = m.names if isinstance(m.names, dict) else dict(enumerate(m.names))
    maps = getattr(m.box, "maps", None)                    # mAP50-95 per class
    maps = [] if maps is None else list(np.asarray(maps).ravel())
    ap50 = getattr(m.box, "ap50", None)
    ap50 = [] if ap50 is None else [float(v) for v in np.asarray(ap50).ravel()]
    rows = []
    for cid in sorted(names):
        r = {"class": names[cid]}
        if cid < len(maps):
            r["mAP50-95"] = float(maps[cid])
        if cid < len(ap50):
            r["mAP50"] = ap50[cid]
        rows.append(r)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="/mnt/d/captures/landmarks/data.yaml")
    ap.add_argument("--model", default="yolov8n.pt",
                    help="预训练权重；没有会自动下载到当前目录")
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="0", help="'0' = 第一块 GPU，'cpu' = 用 CPU")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--patience", type=int, default=30,
                    help="验证指标连续多少轮不涨就早停")
    ap.add_argument("--lr0", type=float, default=0.01,
                    help="初始学习率；从已有权重续训时建议 0.001~0.003")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--project", default="/mnt/d/captures/landmarks/runs")
    ap.add_argument("--name", default="v1")
    ap.add_argument("--cache", default="ram", help="'ram' 缓存图片，小数据集更快")
    args = ap.parse_args()

    if not os.path.exists(args.data):
        print("找不到 data.yaml:", args.data)
        print("先跑 prepare_landmark_dataset.py 生成数据集")
        return 1

    print("=" * 60)
    print("数据:", args.data)
    print("模型:", args.model, "| epochs:", args.epochs, "| imgsz:", args.imgsz,
          "| batch:", args.batch, "| device:", args.device)
    print("=" * 60)

    t0 = time.time()
    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        patience=args.patience,
        lr0=args.lr0,
        seed=args.seed,
        project=args.project,
        name=args.name,
        exist_ok=True,
        cache=args.cache,
        plots=True,          # 出 loss 曲线、混淆矩阵、PR 曲线，答辩要用
        val=True,
        verbose=True,
    )
    train_sec = time.time() - t0

    run_dir = os.path.join(args.project, args.name)
    best = os.path.join(run_dir, "weights", "best.pt")
    if not os.path.exists(best):
        print("训练结束但没找到 best.pt，检查:", run_dir)
        return 1

    # 用 best.pt 在 val 集复算一次，拿干净的指标
    print("\n用 best.pt 复算验证集指标 ...")
    best_model = YOLO(best)
    m = best_model.val(data=args.data, imgsz=args.imgsz, device=args.device,
                       split="val", verbose=False)
    rows = per_class_metrics(m)

    # 权重归档到固定位置，方便后续推理 / 提交
    wdir = "/mnt/d/captures/landmarks/weights"
    os.makedirs(wdir, exist_ok=True)
    dst = os.path.join(wdir, "best_%s.pt" % args.name)
    shutil.copy2(best, dst)

    summary = {
        "run": args.name,
        "data": args.data,
        "model": args.model,
        "epochs": args.epochs,
        "imgsz": args.imgsz,
        "batch": args.batch,
        "lr0": args.lr0,
        "train_seconds": round(train_sec, 1),
        "overall_mAP50": float(getattr(m.box, "map50", 0.0)),
        "overall_mAP50_95": float(getattr(m.box, "map", 0.0)),
        "per_class": rows,
        "weights": dst,
    }
    with open(os.path.join(run_dir, "train_summary.json"), "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    lines = ["# 训练结果（%s）" % args.name, "",
             "- 数据：`%s`" % args.data,
             "- 模型：%s，epochs %d，imgsz %d，batch %d"
             % (args.model, args.epochs, args.imgsz, args.batch),
             "- 训练耗时：%.1f 分钟" % (train_sec / 60),
             "- 整体 mAP50：%.4f，mAP50-95：%.4f"
             % (summary["overall_mAP50"], summary["overall_mAP50_95"]),
             "", "| 类别 | mAP50 | mAP50-95 |", "|---|---|---|"]
    for r in rows:
        lines.append("| %s | %.4f | %.4f |"
                     % (r["class"], r.get("mAP50", 0.0), r.get("mAP50-95", 0.0)))
    lines += ["", "权重：`%s`" % dst]
    with open(os.path.join(run_dir, "train_summary.md"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print("\n" + "\n".join(lines))
    print("\n训练目录:", run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
