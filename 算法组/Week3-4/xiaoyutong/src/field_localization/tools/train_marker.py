"""训练场地标志检测模型（yolov8n）。

用法：
    python tools/train_marker.py                       # 默认 40 epoch，CPU/GPU 自动
    python tools/train_marker.py --epochs 80 --batch 16
    python tools/train_marker.py --device 0            # 有 NVIDIA GPU 时

产物：runs/marker/weights/best.pt（拷到仓库外，README 写获取方式）
"""

import argparse
import os
import sys

def _repo_root():
    """向上找到包含 config/field.yaml 的目录，作为仓库根。"""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(4):
        if os.path.exists(os.path.join(d, "config", "field.yaml")):
            return d
        d = os.path.dirname(d)
    return os.getcwd()

ROOT = _repo_root()
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision2.console import setup_console  # noqa: E402

setup_console()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.join(ROOT, "dataset", "marker_data.yaml"))
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--device", default="cpu", help="cpu 或 0")
    ap.add_argument("--name", default="marker")
    args = ap.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=0,
        project=os.path.join(ROOT, "runs"),
        name=args.name,
        exist_ok=True,
        plots=False,
        verbose=True,
    )

    best = os.path.join(ROOT, "runs", args.name, "weights", "best.pt")
    print("")
    print("训练完成，权重：%s" % best)
    if os.path.exists(best):
        metrics = YOLO(best).val(data=args.data, imgsz=args.imgsz, device=args.device)
        print("")
        print("== 验证集指标 ==")
        print("  Precision   %.4f" % metrics.box.mp)
        print("  Recall      %.4f" % metrics.box.mr)
        print("  mAP50       %.4f" % metrics.box.map50)
        print("  mAP50-95    %.4f" % metrics.box.map)
    return 0


if __name__ == "__main__":
    sys.exit(main())
