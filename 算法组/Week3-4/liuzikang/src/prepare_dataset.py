#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把平铺的 images/all + labels/all 按**视频**划分成 train/val/test，并生成 data.yaml。"""
from __future__ import annotations

import argparse
import shutil
from collections import defaultdict
from pathlib import Path

DEFAULT_ASSIGN = "ball_01=train,ball_03=train,ball_04=train,ball_05=val,ball_02=test"


def imread_ok(p: Path) -> bool:
    data = p.read_bytes()[:4] if p.exists() else b""
    return len(data) > 0


def main() -> int:
    ap = argparse.ArgumentParser(description="按视频划分 YOLO 数据集")
    ap.add_argument("--images", default="dataset/images/all")
    ap.add_argument("--labels", default="dataset/labels/all")
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--assign", default=DEFAULT_ASSIGN,
                    help="视频=集合 的列表，如 ball_03=train,ball_05=val")
    ap.add_argument("--name", default="ball_data.yaml")
    ap.add_argument("--classes", default="ball",
                    help="逗号分隔的类别名，如 LCross,TCross,XCross")
    args = ap.parse_args()

    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    img_dir, lab_dir = Path(args.images), Path(args.labels)
    out = Path(args.out)

    assign: dict[str, str] = {}
    for item in args.assign.split(","):
        item = item.strip()
        if not item:
            continue
        video, split = item.split("=")
        assign[video.strip()] = split.strip()

    images = sorted(img_dir.glob("*.jpg")) + sorted(img_dir.glob("*.png"))
    if not images:
        print("没有图片")
        return 1

    stats = defaultdict(lambda: {"images": 0, "boxes": 0, "empty": 0, "videos": set()})
    split_dirs = {}
    for split in ("train", "val", "test"):
        d_img = out / "images" / split
        d_lab = out / "labels" / split
        d_img.mkdir(parents=True, exist_ok=True)
        d_lab.mkdir(parents=True, exist_ok=True)
        split_dirs[split] = (d_img, d_lab)

    skipped_no_label = []
    videos_with_labels = {p.stem.rsplit("_", 1)[0] for p in lab_dir.glob("*.txt")}
    warned: set[str] = set()
    for img in images:
        video = img.stem.rsplit("_", 1)[0]
        split = assign.get(video)
        if split is None:
            if video in videos_with_labels and video not in warned:
                print(f"[警告] 视频 {video} 有标签但没分配集合，已跳过")
                warned.add(video)
            continue
        lab = lab_dir / (img.stem + ".txt")
        if not lab.exists():
            skipped_no_label.append(img.name)
            continue

        d_img, d_lab = split_dirs[split]
        shutil.copy2(img, d_img / img.name)
        shutil.copy2(lab, d_lab / lab.name)

        lines = [l for l in lab.read_text(encoding="utf-8").splitlines() if l.strip()]
        stats[split]["images"] += 1
        stats[split]["boxes"] += len(lines)
        stats[split]["videos"].add(video)
        if not lines:
            stats[split]["empty"] += 1

    yaml_path = out / args.name
    yaml_text = (
        "# YOLO 数据集配置（由 tools/prepare_dataset.py 生成）\n"
        "# 不写绝对路径：根目录 = 本文件所在目录\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n"
        f"nc: {len(classes)}\n"
        f"names: {classes}\n"
    )
    yaml_path.write_text(yaml_text, encoding="utf-8")

    print("\n=== 划分结果 ===")
    for split in ("train", "val", "test"):
        s = stats[split]
        print(f"{split:5s}: {s['images']:4d} 张, {s['boxes']:4d} 个框, "
              f"空标签 {s['empty']}, 视频 {sorted(s['videos'])}")
    if skipped_no_label:
        print(f"\n[跳过] {len(skipped_no_label)} 张没有标签文件（还没标）")
    print(f"\ndata.yaml -> {yaml_path.resolve()}")

    summary = out / "dataset_summary.md"
    with summary.open("w", encoding="utf-8") as f:
        f.write("# 足球检测数据集说明\n\n")
        f.write("| 集合 | 图片数 | 标注框数 | 空标签 | 来源视频 |\n|---|---|---|---|---|\n")
        for split in ("train", "val", "test"):
            s = stats[split]
            f.write(f"| {split} | {s['images']} | {s['boxes']} | {s['empty']} | "
                    f"{', '.join(sorted(s['videos']))} |\n")
        f.write("\n- 采集设备：单目 USB 相机，1280×720，30 fps\n")
        f.write("- 抽帧方式：每 15 帧取 1 张，自动丢弃模糊帧和重复帧\n")
        f.write("- 划分方式：按**视频片段**整体划分，避免相邻帧泄漏\n")
        f.write("- 类别：`ball`（class_id = 0），单类别\n")
    print(f"数据集说明 -> {summary.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
