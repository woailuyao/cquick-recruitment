from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]


def main():
    data_yaml = ROOT / "dataset" / "data.yaml"
    weight = ROOT / "runs" / "ball_v1" / "weights" / "best.pt"

    if not data_yaml.exists():
        raise SystemExit(f"找不到 data.yaml: {data_yaml}")
    if not weight.exists():
        raise SystemExit(f"找不到 best.pt: {weight}")

    model = YOLO(str(weight))
    metrics = model.val(
        data=str(data_yaml),
        split="test",
        imgsz=640,
        device=0,
        project=str(ROOT / "runs"),
        name="val_v1",
    )

    print("--- test metrics ---")
    for key, value in metrics.results_dict.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()