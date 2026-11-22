import argparse
import shutil
from collections import Counter
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_video_set(value: str):
    return {item.strip() for item in value.split(",") if item.strip()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="dataset",
                        help="dataset 目录，里面应有 images/all")
    parser.add_argument("--images", default=None,
                        help="标注图片目录，默认 dataset/images/all")
    parser.add_argument("--labels", default=None,
                        help="标签目录；默认与图片同目录（LabelImg 默认）")
    parser.add_argument("--train", default="1,2,3,4")
    parser.add_argument("--val", default="5")
    parser.add_argument("--test", default="6")
    args = parser.parse_args()

    dataset = Path(args.dataset)
    images_dir = Path(args.images) if args.images else dataset / "images" / "all"
    labels_dir = Path(args.labels) if args.labels else images_dir

    mapping = {}
    for split, value in (("train", args.train), ("val", args.val), ("test", args.test)):
        for video in parse_video_set(value):
            if video in mapping:
                raise SystemExit(f"视频 {video} 被分到了多个集合")
            mapping[video] = split

    if not images_dir.exists():
        raise SystemExit(f"图片目录不存在: {images_dir}")

    counts = Counter()
    missing_labels = []
    bad_labels = []

    for image_path in sorted(images_dir.iterdir()):
        if image_path.suffix.lower() not in IMAGE_SUFFIXES:
            continue

        # 1_000010.jpg -> video=1
        video = image_path.stem.split("_")[0]
        split = mapping.get(video)
        if split is None:
            print(f"[跳过] 未配置视频: {image_path.name}")
            continue

        label_path = labels_dir / f"{image_path.stem}.txt"
        if not label_path.exists():
            missing_labels.append(image_path.name)
            continue

        lines = [line.strip() for line in label_path.read_text(encoding="utf-8").splitlines() if line.strip()]
        if not lines:
            bad_labels.append(f"{label_path.name}: 空标签")
            continue

        valid = True
        for line in lines:
            parts = line.split()
            if len(parts) != 5:
                bad_labels.append(f"{label_path.name}: 不是 5 列")
                valid = False
                break
            if parts[0] != "0":
                bad_labels.append(f"{label_path.name}: class_id 不是 0")
                valid = False
                break
            try:
                values = [float(x) for x in parts[1:]]
            except ValueError:
                bad_labels.append(f"{label_path.name}: 坐标不是数字")
                valid = False
                break
            if any(x < 0 or x > 1 for x in values):
                bad_labels.append(f"{label_path.name}: 坐标越界")
                valid = False
                break
        if not valid:
            continue

        dst_image = dataset / "images" / split / image_path.name
        dst_label = dataset / "labels" / split / label_path.name
        dst_image.parent.mkdir(parents=True, exist_ok=True)
        dst_label.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image_path, dst_image)
        shutil.copy2(label_path, dst_label)
        counts[split] += 1

    print("--- 划分结果 ---")
    for split in ("train", "val", "test"):
        print(f"{split}: {counts[split]} 张")

    if missing_labels:
        print(f"缺少标签 {len(missing_labels)} 张，例如: {missing_labels[:5]}")
    if bad_labels:
        print(f"标签格式错误 {len(bad_labels)} 个，例如: {bad_labels[:5]}")
    if missing_labels or bad_labels:
        print("请先修复上述问题，再重新运行本脚本。")

    if sum(counts.values()) < 200:
        print("警告: 当前有效图片少于 200 张。")


if __name__ == "__main__":
    main()