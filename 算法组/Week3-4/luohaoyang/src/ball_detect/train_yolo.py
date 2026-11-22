from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]


def main():
    data_yaml = ROOT / "dataset" / "data.yaml"
    if not data_yaml.exists():
        raise SystemExit(f"找不到 data.yaml: {data_yaml}")

    model_path = ROOT / "yolov8n.pt"
    if not model_path.exists():
        raise SystemExit(f"找不到预训练权重: {model_path}")

    model = YOLO(str(model_path))
    model.train(
        data=str(data_yaml),
        epochs=50,
        imgsz=640,
        batch=8,
        workers=4,
        device=0,
        project=str(ROOT / "runs"),
        name="ball_v1",
    )


if __name__ == "__main__":
    main()