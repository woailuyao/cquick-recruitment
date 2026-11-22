# -*- coding: utf-8 -*-
"""任务一训练：YOLOv8n 单类别足球检测（细则 1.6 的基线参数）。

基线：model=yolov8n.pt, imgsz=640, epochs=50, device=0(GPU)
产出：runs/detect/ball_v1/ 下的权重与曲线；best.pt 另存到 weights/best_ball.pt

用法（项目根目录）：
    python tools/train.py
    python tools/train.py --epochs 80 --batch 8 --name ball_v2
"""

from __future__ import annotations

import argparse
import csv
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import DATASET_DIR, RUNS_DIR, WEIGHTS_DIR, rel     # noqa: E402


def summarize_results(results_csv: Path) -> dict:
    """从 results.csv 读最后一行的指标。"""
    if not results_csv.exists():
        return {}
    with results_csv.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    last = {k.strip(): v for k, v in rows[-1].items()}
    keys = {
        "precision": "metrics/precision(B)",
        "recall": "metrics/recall(B)",
        "map50": "metrics/mAP50(B)",
        "map50_95": "metrics/mAP50-95(B)",
    }
    out = {}
    for name, col in keys.items():
        if col in last:
            try:
                out[name] = float(last[col])
            except ValueError:
                pass
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="训练足球检测模型")
    ap.add_argument("--data", default=str(DATASET_DIR / "data.yaml"))
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="0")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--name", default="ball_v1")
    args = ap.parse_args()

    data_path = Path(args.data)
    if not data_path.is_absolute():
        data_path = ROOT / data_path
    if not data_path.exists():
        print(f"没找到 {rel(data_path)}，请先跑 tools/split_dataset.py")
        return 1

    weight_path = Path(args.model)
    if not weight_path.is_absolute() and not weight_path.exists():
        cand = WEIGHTS_DIR / args.model
        if cand.exists():
            weight_path = cand

    from ultralytics import YOLO
    print("=" * 74)
    print(f"训练配置：model={weight_path.name if weight_path.exists() else args.model} "
          f"data={rel(data_path)} imgsz={args.imgsz} epochs={args.epochs} "
          f"batch={args.batch} device={args.device}")
    print("=" * 74)

    model = YOLO(str(weight_path) if weight_path.exists() else args.model)
    results = model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(RUNS_DIR),
        name=args.name,
        exist_ok=True,
        plots=True,
        val=True,
        verbose=True,
    )

    run_dir = Path(results.save_dir) if getattr(results, "save_dir", None) else RUNS_DIR / args.name
    best = run_dir / "weights" / "best.pt"
    if best.exists():
        WEIGHTS_DIR.mkdir(parents=True, exist_ok=True)
        # 只把「足球」那一路的权重叫 best_ball.pt，其余按训练名另存，
        # 免得训标志模型时把任务一的足球权重覆盖掉。
        dst_name = "best_ball.pt" if args.name.startswith("ball") else f"best_{args.name}.pt"
        dst = WEIGHTS_DIR / dst_name
        shutil.copy2(best, dst)
        print(f"最佳权重已复制：{rel(dst)}（{dst.stat().st_size / 1e6:.2f} MB）")

    metrics = summarize_results(run_dir / "results.csv")
    print("-" * 74)
    if metrics:
        print("最终指标（最后一轮）：")
        print(f"  Precision  {metrics.get('precision', float('nan')):.4f}")
        print(f"  Recall     {metrics.get('recall', float('nan')):.4f}")
        print(f"  mAP50      {metrics.get('map50', float('nan')):.4f}")
        print(f"  mAP50-95   {metrics.get('map50_95', float('nan')):.4f}")
    print(f"训练目录   ：{rel(run_dir)}")
    print("下一步：python tools/evaluate.py")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
