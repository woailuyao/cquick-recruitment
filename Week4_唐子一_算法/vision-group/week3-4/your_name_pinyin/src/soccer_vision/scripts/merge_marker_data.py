#!/usr/bin/env python3
"""把 marker 数据集的 train/valid/test 全部合并进 train_all。

用法：
    python3 merge_marker_data.py --root dataset/marker-dataset.v1i.yolov8

说明：
    - 清空并重建 train_all/images 与 train_all/labels；
    - 将 train、valid、test 三个子集的图片和标签全部复制进 train_all；
    - 同时生成 data_all.yaml（全量训练用，train=val=train_all）。
"""

import argparse
import shutil
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="数据集根目录（含 train/valid/test 子目录）")
    args = ap.parse_args()

    root = Path(args.root).resolve()
    if not root.exists():
        raise SystemExit(f"[FAIL] 目录不存在: {root}")

    train_all = root / "train_all"
    img_dir = train_all / "images"
    lbl_dir = train_all / "labels"

    # 清空重建
    if train_all.exists():
        shutil.rmtree(train_all)
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    total_img = 0
    total_lbl = 0
    for split in ("train", "valid", "test"):
        s_img = root / split / "images"
        s_lbl = root / split / "labels"
        if not s_img.exists():
            continue
        for f in sorted(s_img.iterdir()):
            if f.suffix.lower() in (".jpg", ".jpeg", ".png"):
                shutil.copy2(f, img_dir / f.name)
                total_img += 1
        if s_lbl.exists():
            for f in sorted(s_lbl.iterdir()):
                if f.suffix.lower() == ".txt":
                    shutil.copy2(f, lbl_dir / f.name)
                    total_lbl += 1

    print(f"[OK] 合并完成：图片 {total_img} 张，标签 {total_lbl} 个 -> {train_all}")

    # 生成 data_all.yaml
    data_yaml = root / "data_all.yaml"
    rel = root.name
    content = (
        f"# 全量训练用 data.yaml（自动生成）\n"
        f"path: dataset/{rel}\n"
        f"train: train_all/images\n"
        f"val: train_all/images\n"
        f"nc: 2\n"
        f"names: ['LCross', 'TCross']\n"
    )
    data_yaml.write_text(content, encoding="utf-8")
    print(f"[OK] 已生成 {data_yaml}")


if __name__ == "__main__":
    main()
