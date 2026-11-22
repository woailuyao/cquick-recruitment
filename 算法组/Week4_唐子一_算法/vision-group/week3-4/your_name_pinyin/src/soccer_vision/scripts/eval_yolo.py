#!/usr/bin/env python3
"""独立测试集评估：输出 Precision / Recall / mAP50 / mAP50-95。

用法：
    python eval_yolo.py --weights runs/ball/weights/best.pt --data soccer_dataset/data.yaml
"""

import argparse

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True, help="best.pt 路径")
    ap.add_argument("--data", required=True, help="data.yaml 路径")
    ap.add_argument("--device", default="0")
    args = ap.parse_args()

    model = YOLO(args.weights)
    metrics = model.val(data=args.data, split="test", device=args.device)

    print("\n========== 独立测试集指标 ==========")
    print(f"Precision: {metrics.box.mp:.4f}")
    print(f"Recall:    {metrics.box.mr:.4f}")
    print(f"mAP50:     {metrics.box.map50:.4f}")
    print(f"mAP50-95:  {metrics.box.map:.4f}")
    print("====================================")


if __name__ == "__main__":
    main()
