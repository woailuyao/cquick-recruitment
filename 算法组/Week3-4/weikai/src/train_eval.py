# -*- coding: utf-8 -*-
"""标注完成、verify_labels.py 通过后，训练 yolov8n 并在独立测试集评估。"""
import argparse
import csv
from pathlib import Path

from ultralytics import YOLO

ap = argparse.ArgumentParser()
ap.add_argument("--data", default="soccer_dataset/data.yaml")
ap.add_argument("--weights", default="yolov8n.pt")
ap.add_argument("--epochs", type=int, default=50)
ap.add_argument("--imgsz", type=int, default=640)
ap.add_argument("--device", default="0")
ap.add_argument("--batch", type=int, default=16)
ap.add_argument("--name", default="ball_yolov8n")
a = ap.parse_args()

# 用绝对路径，避免 Ultralytics 把相对 project 拼成 runs/detect/runs/detect
BASE = Path.cwd() / "runs"
PROJECT = str(BASE)

model = YOLO(a.weights)
results = model.train(data=a.data, epochs=a.epochs, imgsz=a.imgsz,
                      device=a.device, batch=a.batch,
                      project=PROJECT, name=a.name, exist_ok=True)

# 直接从训练结果拿 best 权重路径，不靠拼路径猜
save_dir = Path(results.save_dir)
best = save_dir / "weights" / "best.pt"
print("\n[INFO] save_dir:", save_dir.resolve())
print("[INFO] 最佳权重:", best.resolve())
if not best.exists():
    raise SystemExit("[FAIL] 找不到 best.pt: %s" % best)

# 独立测试集评估（test 集合从未参与训练/验证）
eval_model = YOLO(str(best))
metrics = eval_model.val(data=a.data, split="test", imgsz=a.imgsz,
                         device=a.device, project=PROJECT,
                         name=a.name + "_test", exist_ok=True)

r = metrics.results_dict
row = {
    "model": "yolov8n", "epochs": a.epochs, "imgsz": a.imgsz,
    "precision": round(r.get("metrics/precision(B)", 0), 5),
    "recall": round(r.get("metrics/recall(B)", 0), 5),
    "mAP50": round(r.get("metrics/mAP50(B)", 0), 5),
    "mAP50-95": round(r.get("metrics/mAP50-95(B)", 0), 5),
}
out = save_dir / "test_metrics.csv"
with out.open("w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=list(row))
    w.writeheader(); w.writerow(row)

print("\n===== 独立测试集指标 =====")
for k, v in row.items():
    print("  %-12s %s" % (k, v))
print("指标已保存:", out.resolve())
