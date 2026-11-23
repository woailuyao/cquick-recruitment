# -*- coding: utf-8 -*-
"""
半自动预标注（用现有模型先出框，人工只做修正）

为什么值得做：
    实测用 COCO 的 `sports ball` 类在本人实拍数据上能检出 **90%** 的图片。
    也就是说 200 张里约 180 张的框已经基本对了，人工只需要
    改框、补漏、删误检——比从零手画快好几倍。

输出格式：YOLO 检测格式，每张图一个同名 .txt，写在与图片**同级**目录。
          （后面 prepare_dataset.py 会整理成 images/ labels/ 结构）

用法：

    # 足球（用 COCO 预训练模型，class 0 = ball）
    python scripts/prelabel.py --src dataset_raw --task ball

    # 场地标志（需要你自己的 4 类模型）
    python scripts/prelabel.py --src dataset_marker --task marker \
        --weights weights/marker_best.pt

注意：预标注结果**必须人工复核**。细则明确禁止用他人权重冒充自己的训练结果，
      COCO 模型只用来省手工，最终提交的权重必须是你在自己数据上训出来的。
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

sys.path.insert(0, ROOT)
from src.config import FIELD_MARKER_CLASSES  # noqa: E402
sys.path.insert(0, ROOT)

MARKER_ORDER = FIELD_MARKER_CLASSES          # 现场实际存在的类别，见 src/config.py


def norm_class(name: str) -> str:
    s = name.strip().lower().replace("-", "_").replace(" ", "_")
    table = {
        "sports_ball": "ball", "ball": "ball",
        "lcross": "LCross", "l_cross": "LCross", "l": "LCross",
        "tcross": "TCross", "t_cross": "TCross", "t": "TCross",
        "xcross": "XCross", "x_cross": "XCross", "x": "XCross",
        "penalitypoint": "PenaltyPoint", "penaltypoint": "PenaltyPoint",
        "penalty_point": "PenaltyPoint",
    }
    return table.get(s, name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(ROOT, "dataset_raw"))
    ap.add_argument("--weights", default=os.path.join(ROOT, "weights", "yolov8n.pt"))
    ap.add_argument("--task", choices=["ball", "marker"], default="ball")
    ap.add_argument("--conf", type=float, default=0.20,
                    help="预标注阈值。宁可多出框让人删，也别漏（漏了容易忘记补）")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--overwrite", action="store_true",
                    help="覆盖已存在的 txt（默认跳过，保留人工改过的结果）")
    ap.add_argument("--only-empty", action="store_true",
                    help="只处理 txt 缺失或为空的图，用于给漏检的困难样本补框")
    ap.add_argument("--preview", type=int, default=12, help="生成多少张预览图供抽查")
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)
    names = model.names

    # 目标类别集合
    if args.task == "ball":
        wanted = {"ball"}
    else:
        wanted = set(MARKER_ORDER)

    imgs = sorted(glob.glob(os.path.join(args.src, "*", "*.jpg"))
                  + glob.glob(os.path.join(args.src, "*", "*.png")))
    if not imgs:
        print(f"[错误] {args.src} 下没有图片（期望结构: {args.src}/<片段>/*.jpg）")
        return 1

    if args.only_empty:
        kept = []
        for p in imgs:
            tp = os.path.splitext(p)[0] + ".txt"
            if not os.path.isfile(tp) or os.path.getsize(tp) == 0:
                kept.append(p)
        print(f"[--only-empty] 只处理空标注图: {len(kept)}/{len(imgs)} 张")
        imgs = kept
        if not imgs:
            print("没有需要补标的空图。")
            return 0

    print(f"预标注 {len(imgs)} 张，模型 {os.path.basename(args.weights)}，"
          f"任务 {args.task}，conf {args.conf}")
    print("=" * 62)

    n_written = n_skipped = n_empty = 0
    total_boxes = 0
    rows = []
    preview_dir = os.path.join(ROOT, "snapshots", "prelabel_preview")
    os.makedirs(preview_dir, exist_ok=True)
    preview_budget = args.preview

    for i, p in enumerate(imgs, 1):
        stem = os.path.splitext(p)[0]
        txt = stem + ".txt"
        # 只有**非空**标签才算"已标注"。
        # 空 txt 意味着上次也没检出目标，正是 --only-empty 要补的对象，
        # 不能因为文件存在就跳过。
        has_content = os.path.exists(txt) and os.path.getsize(txt) > 0
        if has_content and not args.overwrite:
            n_skipped += 1
            continue

        img = cv2.imread(p)
        if img is None:
            continue
        h, w = img.shape[:2]
        res = model.predict(img, imgsz=args.imgsz, conf=args.conf,
                            device=args.device, verbose=False)[0]

        lines = []
        boxes_draw = []
        for j in range(len(res.boxes)):
            cid = int(res.boxes.cls[j].item())
            label = norm_class(names.get(cid, str(cid)))
            if label not in wanted:
                continue
            x1, y1, x2, y2 = res.boxes.xyxy[j].tolist()
            x1 = max(0.0, min(x1, w - 1)); x2 = max(0.0, min(x2, w - 1))
            y1 = max(0.0, min(y1, h - 1)); y2 = max(0.0, min(y2, h - 1))
            bw, bh = x2 - x1, y2 - y1
            if bw < 4 or bh < 4:
                continue
            k = 0 if label == "ball" else MARKER_ORDER.index(label)
            cx = (x1 + x2) / 2 / w
            cy = (y1 + y2) / 2 / h
            lines.append(f"{k} {cx:.6f} {cy:.6f} {bw / w:.6f} {bh / h:.6f}")
            boxes_draw.append((int(x1), int(y1), int(x2), int(y2), label,
                               float(res.boxes.conf[j].item())))

        with open(txt, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        n_written += 1
        total_boxes += len(lines)
        if not lines:
            n_empty += 1
        rows.append({"file": os.path.relpath(p, args.src),
                     "boxes": len(lines),
                     "classes": ",".join(sorted({b[4] for b in boxes_draw}))})

        if preview_budget > 0 and boxes_draw:
            vis = img.copy()
            for x1, y1, x2, y2, label, cf in boxes_draw:
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(vis, f"{label} {cf:.2f}", (x1, max(14, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.imwrite(os.path.join(preview_dir,
                                     os.path.basename(os.path.dirname(p)) + "_" + os.path.basename(p)),
                        vis)
            preview_budget -= 1

        if i % 100 == 0:
            print(f"  已处理 {i}/{len(imgs)} ...")

    csv_path = os.path.join(args.src, "prelabel_report.csv")
    if rows:
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            wtr = csv.DictWriter(f, fieldnames=["file", "boxes", "classes"])
            wtr.writeheader()
            wtr.writerows(rows)

    print("=" * 62)
    print(f"写入标签 {n_written} 张，跳过（已存在）{n_skipped} 张")
    print(f"共生成 {total_boxes} 个框，平均每张 {total_boxes / max(1, n_written):.2f} 个")
    print(f"其中 {n_empty} 张**没有检出任何目标**（这些要人工确认是不是漏检）")
    print(f"报告: {csv_path}")
    print(f"预览图: {preview_dir}")
    print()
    print("下一步：人工复核")
    print(f"  .\\.venv\\Scripts\\python.exe scripts\\label_tool.py --src {os.path.relpath(args.src, ROOT)} --task {args.task}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
