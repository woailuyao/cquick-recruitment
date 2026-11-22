#!/usr/bin/env python3
"""YOLO 训练脚本（足球检测 / 场地标志检测通用）。

基线：yolov8n.pt, imgsz=640, epochs=50。
用法（足球）：
    python train_yolo.py --data soccer_dataset/data.yaml --name ball --epochs 50 --device 0
用法（标志）：
    python train_yolo.py --data marker_dataset/data.yaml --name marker --epochs 50 --device 0
"""

import argparse

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="data.yaml 路径")
    ap.add_argument("--name", default="train", help="训练任务名（保存目录名）")
    ap.add_argument("--model", default="yolov8n.pt", help="预训练权重")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="0", help="0 或 cpu")
    args = ap.parse_args()

    model = YOLO(args.model)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        name=args.name,
        project="runs",
    )
    print(f"训练完成，最佳权重保存在 runs/{args.name}/weights/best.pt")
    return results


if __name__ == "__main__":
    main()
