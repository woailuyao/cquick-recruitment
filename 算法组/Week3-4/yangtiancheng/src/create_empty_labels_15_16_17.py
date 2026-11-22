from collections import Counter
from pathlib import Path
import sys

project = Path.home() / "vision_ws/k1_soccer_vision"
image_root = project / "datasets/ball/images/train/all"
label_root = project / "datasets/ball/labels/train"
sources = ("15", "16", "17")
extensions = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

records = []
source_counts = Counter()

for source in sources:
    source_dir = image_root / source
    if not source_dir.is_dir():
        print(f"错误：来源目录不存在：{source_dir}", file=sys.stderr)
        sys.exit(1)

    images = sorted(
        path for path in source_dir.rglob("*")
        if path.is_file() and path.suffix.lower() in extensions
    )
    if not images:
        print(f"错误：来源 {source} 中没有图片", file=sys.stderr)
        sys.exit(1)

    for image in images:
        records.append((source, image))
        source_counts[source] += 1

stems = Counter(image.stem for _, image in records)
duplicates = sorted(stem for stem, count in stems.items() if count > 1)
if duplicates:
    print("错误：15/16/17 中存在重名图片，未创建任何标签：", file=sys.stderr)
    for stem in duplicates[:50]:
        print(f"  {stem}", file=sys.stderr)
    sys.exit(1)

conflicts = []
for source, image in records:
    label = label_root / f"{image.stem}.txt"
    if label.exists() and label.stat().st_size > 0:
        conflicts.append((source, image, label))

if conflicts:
    print("错误：发现已有非空标签，未创建任何新标签：", file=sys.stderr)
    for source, image, label in conflicts[:50]:
        print(f"  来源 {source}: {image.name} -> {label.name}", file=sys.stderr)
    sys.exit(1)

label_root.mkdir(parents=True, exist_ok=True)
created = Counter()
existing_empty = Counter()

for source, image in records:
    label = label_root / f"{image.stem}.txt"
    if label.exists():
        existing_empty[source] += 1
    else:
        label.touch(exist_ok=False)
        created[source] += 1

print("===== 空标签创建结果 =====")
for source in sources:
    print(
        f"来源 {source}: 图片 {source_counts[source]}, "
        f"新建空标签 {created[source]}, 已有空标签 {existing_empty[source]}"
    )

selected_labels = [label_root / f"{image.stem}.txt" for _, image in records]
nonempty = [label for label in selected_labels if label.stat().st_size > 0]

print("\n===== 最终核对 =====")
print(f"图片总数: {len(records)}")
print(f"对应标签总数: {sum(label.exists() for label in selected_labels)}")
print(f"非空标签数: {len(nonempty)}")
print("结果:", "正常" if not nonempty else "异常")
