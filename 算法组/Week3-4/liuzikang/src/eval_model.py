#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在独立测试集上评估足球检测模型，并生成"预测框回画图"用于挑误检/漏检。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


def load_yolo_label(txt: Path, w: int, h: int):
    boxes = []
    if not txt.exists():
        return boxes
    for line in txt.read_text(encoding="utf-8").splitlines():
        p = line.split()
        if len(p) < 5:
            continue
        cx, cy, bw, bh = map(float, p[1:5])
        boxes.append((int((cx - bw / 2) * w), int((cy - bh / 2) * h),
                      int((cx + bw / 2) * w), int((cy + bh / 2) * h)))
    return boxes


def main() -> int:
    ap = argparse.ArgumentParser(description="评估足球检测模型")
    ap.add_argument("--model", default="runs/detect/train/weights/best.pt")
    ap.add_argument("--data", default="dataset/ball_data.yaml")
    ap.add_argument("--split", default="test", choices=["train", "val", "test"])
    ap.add_argument("--imgsz", type=int, default=512)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--out", default="runs/eval")
    ap.add_argument("--max-preview", type=int, default=40)
    args = ap.parse_args()

    if not Path(args.model).exists():
        print(f"找不到模型：{args.model}")
        return 1

    out = Path(args.out)
    pred_dir = out / "predictions"
    pred_dir.mkdir(parents=True, exist_ok=True)

    model = YOLO(args.model)
    metrics = model.val(
        data=args.data, split=args.split, imgsz=args.imgsz,
        conf=args.conf, project=str(out), name="val", exist_ok=True,
        plots=False, verbose=False,
    )

    rec = {
        "model": str(args.model), "split": args.split,
        "imgsz": args.imgsz, "conf_threshold": args.conf,
        "precision": float(metrics.box.mp), "recall": float(metrics.box.mr),
        "map50": float(metrics.box.map50), "map50_95": float(metrics.box.map),
    }
    (out / "metrics.json").write_text(json.dumps(rec, ensure_ascii=False, indent=2),
                                      encoding="utf-8")
    print("\n=== 指标 ===")
    for k, v in rec.items():
        print(f"  {k}: {v}" if isinstance(v, str) else f"  {k}: {v:.4f}")

    img_dir = Path("dataset") / "images" / args.split
    lab_dir = Path("dataset") / "labels" / args.split
    images = sorted(img_dir.glob("*.jpg"))[: args.max_preview]
    print(f"\n生成 {len(images)} 张预测图……")
    for img_path in images:
        data = np.fromfile(str(img_path), dtype=np.uint8)
        img = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if img is None:
            continue
        h, w = img.shape[:2]
        res = model.predict(img, imgsz=args.imgsz, conf=args.conf,
                            verbose=False, device="cpu")[0]
        for b in res.boxes:
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
            cf = float(b.conf[0])
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f"ball {cf:.2f}", (x1, max(16, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        for (x1, y1, x2, y2) in load_yolo_label(lab_dir / (img_path.stem + ".txt"), w, h):
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 1)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if ok:
            buf.tofile(str(pred_dir / img_path.name))

    print(f"预测图：{pred_dir.resolve()}")
    print("\n挑案例：")
    print("  漏检 = 有黄色框但没绿色框；误检 = 有绿色框但没黄色框")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
