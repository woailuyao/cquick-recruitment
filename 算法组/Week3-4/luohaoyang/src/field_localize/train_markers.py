from pathlib import Path
import argparse

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[2]
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--name", default="markers_v2")
    args = parser.parse_args()

    data_yaml = ROOT / "dataset" / "data.yaml"
    model_path = ROOT / "yolov8n.pt"

    if not data_yaml.exists():
        raise SystemExit(f"找不到 data.yaml: {data_yaml}")
    if not model_path.exists():
        raise SystemExit(f"找不到预训练权重: {model_path}")

    model = YOLO(str(model_path))
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=4,
        device=0,
        project=str(ROOT / "runs"),
        name=args.name,
    )


if __name__ == "__main__":
    main()