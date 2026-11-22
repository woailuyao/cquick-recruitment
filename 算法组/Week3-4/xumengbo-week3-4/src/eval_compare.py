"""对比 best.pt 与 last.pt 在 test 集上的指标，判断早停是否选错权重。"""
from ultralytics import YOLO

DATA_YAML = r"D:\W34_dataset\ball\ball_data.yaml"
WEIGHT_DIR = r"D:\W3-4_许勐搏_单目定位\runs\ball\train\weights"
PROJECT = r"D:\W3-4_许勐搏_单目定位\runs\ball"


def main():
    lines = []
    for w in ["best.pt", "last.pt"]:
        model = YOLO(f"{WEIGHT_DIR}\\{w}")
        metrics = model.val(
            data=DATA_YAML,
            split="test",
            imgsz=640,
            batch=8,
            device=0,
            workers=0,
            project=PROJECT,
            name=f"eval_{w[:-3]}",
            exist_ok=True,
        )
        line = (f"{w}: P={metrics.box.p[0]:.4f} R={metrics.box.r[0]:.4f} "
                f"mAP50={metrics.box.map50:.4f} mAP50-95={metrics.box.map:.4f}")
        print(line)
        lines.append(line)

    with open(r"D:\eval_compare.txt", "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    main()
