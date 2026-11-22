#!/usr/bin/env python3
"""Train the field-marker YOLO detector (spec baseline: yolov8n, 640, 50 ep).

Usage:
    python3 train_markers.py --data ./marker_dataset/data.yaml
    python3 train_markers.py --data ./marker_dataset/data.yaml --epochs 50

Prints the resulting best.pt path when training finishes. Pretrained-weight
initialisation is allowed by the spec; training data must be your own.
"""
import argparse
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="data.yaml from split_dataset.py")
    ap.add_argument("--weights", default="yolov8n.pt",
                    help="pretrained weights to initialise from")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--project", default="runs_markers")
    ap.add_argument("--name", default="markers_v1")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("ultralytics not installed: pip install ultralytics")

    model = YOLO(args.weights)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=args.project,
        name=args.name,
    )
    save_dir = Path(results.save_dir)
    best = save_dir / "weights" / "best.pt"
    print(f"training done. save_dir: {save_dir}")
    print(f"best weights: {best}")
    print("next: python3 evaluate_markers.py "
          f"--weights {best} --data {args.data}")


if __name__ == "__main__":
    main()
