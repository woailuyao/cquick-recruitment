from pathlib import Path
from collections import Counter
import shutil
import sys

project = Path.home() / "vision_ws/k1_soccer_vision"
image_root = project / "datasets/ball/images/train/all"
label_root = project / "datasets/ball/labels/train"
target = project / "datasets/ball_split_v3"
extensions = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

# 来源10包含已经观察到的困难背景，整体进入训练集。
# 来源1和14退出训练集，作为v3的新正样本测试来源。
positive_sources = {
    "train": ("2", "3", "4", "5", "6", "9", "10", "11"),
    "val": ("12",),
    "test": ("1", "14"),
}
negative_sources = {
    "train": ("15",),
    "val": ("16",),
    "test": ("17",),
}

expected = {
    "train": {"images": 275, "backgrounds": 115, "objects": 160},
    "val": {"images": 67, "backgrounds": 34, "objects": 33},
    "test": {"images": 77, "backgrounds": 41, "objects": 36},
}

if target.exists() and any(target.rglob("*")):
    print(f"错误：目标目录已有内容，未覆盖：{target}", file=sys.stderr)
    sys.exit(1)

records = []
errors = []

def source_images(source):
    directory = image_root / source
    if not directory.is_dir():
        errors.append(f"来源目录不存在：{directory}")
        return []
    images = sorted(
        p for p in directory.rglob("*")
        if p.is_file() and p.suffix.lower() in extensions
    )
    if not images:
        errors.append(f"来源 {source} 没有图片")
    return images

for split, sources in positive_sources.items():
    for source in sources:
        for image in source_images(source):
            label = label_root / f"{image.stem}.txt"
            if not label.is_file():
                # 来源目录还包含尚未标注的抽帧；v3只纳入已有标注配对。
                continue
            if label.stat().st_size == 0:
                errors.append(f"正样本来源 {source} 标签为空：{label.name}")
                continue
            records.append((split, source, "positive", image, label))

for split, sources in negative_sources.items():
    for source in sources:
        for image in source_images(source):
            label = label_root / f"{image.stem}.txt"
            if not label.is_file():
                errors.append(f"负样本来源 {source} 缺少标签：{image.name}")
                continue
            if label.stat().st_size != 0:
                errors.append(f"负样本来源 {source} 标签非空：{label.name}")
                continue
            records.append((split, source, "background", image, label))

name_counts = Counter(image.name.lower() for _, _, _, image, _ in records)
duplicates = sorted(name for name, count in name_counts.items() if count > 1)
if duplicates:
    errors.append("存在重名图片：" + ", ".join(duplicates[:30]))

if errors:
    print("===== 检查失败，未创建v3 =====", file=sys.stderr)
    for error in errors:
        print(error, file=sys.stderr)
    sys.exit(1)

for split in ("train", "val", "test"):
    (target / "images" / split).mkdir(parents=True, exist_ok=True)
    (target / "labels" / split).mkdir(parents=True, exist_ok=True)

for split, source, kind, image, label in records:
    shutil.copy2(image, target / "images" / split / image.name)
    shutil.copy2(label, target / "labels" / split / label.name)

(target / "ball.yaml").write_text(
    f"path: {target}\n"
    "train: images/train\n"
    "val: images/val\n"
    "test: images/test\n\n"
    "names:\n"
    "  0: ball\n",
    encoding="utf-8",
)

(target / "split_sources.txt").write_text(
    "positive train: 2, 3, 4, 5, 6, 9, 10, 11\n"
    "positive val: 12\n"
    "positive test: 1, 14\n"
    "negative train: 15\n"
    "negative val: 16\n"
    "negative test: 17\n",
    encoding="utf-8",
)

print("===== ball_split_v3 核对 =====")
all_ok = True
for split in ("train", "val", "test"):
    images = [
        p for p in (target / "images" / split).glob("*")
        if p.is_file() and p.suffix.lower() in extensions
    ]
    labels = list((target / "labels" / split).glob("*.txt"))
    backgrounds = sum(p.stat().st_size == 0 for p in labels)
    objects = len(labels) - backgrounds
    actual = {
        "images": len(images),
        "backgrounds": backgrounds,
        "objects": objects,
    }
    ok = len(images) == len(labels) and actual == expected[split]
    all_ok &= ok
    print(
        f"{split}: 图片 {len(images)}, 标签 {len(labels)}, "
        f"有球 {objects}, 无球 {backgrounds} "
        f"[{'正常' if ok else '异常'}]"
    )

print(f"配置文件: {target / 'ball.yaml'}")
print("结果:", "v3创建完成" if all_ok else "数量异常，请停止训练")
if not all_ok:
    sys.exit(1)
