#!/usr/bin/env python3
"""检查 YOLO 检测数据集标签，不修改任何图片或标签。"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import yaml

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def names_to_dict(names):
    if isinstance(names, dict):
        return {str(k): str(v) for k, v in names.items()}
    return {str(i): str(v) for i, v in enumerate(names or [])}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    cfg = yaml.safe_load(args.data.read_text(encoding="utf-8")) or {}
    names = names_to_dict(cfg.get("names"))
    nc = int(cfg.get("nc", len(names)))
    dataset_root = Path(cfg.get("path", args.data.parent))
    if not dataset_root.is_absolute():
        dataset_root = (args.data.parent / dataset_root).resolve()
    issues = []
    splits = {}
    for split in ("train", "val", "test"):
        rel = cfg.get(split)
        if not rel:
            continue
        image_dir = Path(rel)
        if not image_dir.is_absolute():
            image_dir = dataset_root / image_dir
        image_dir = image_dir.resolve()
        if image_dir.is_file() and image_dir.suffix=='.txt':
            images=[Path(v) if Path(v).is_absolute() else image_dir.parent/v.removeprefix('./') for v in image_dir.read_text(encoding='utf-8').splitlines() if v.strip()]
        else:
            images = sorted(x for x in image_dir.rglob("*") if x.suffix.lower() in IMAGE_SUFFIXES) if image_dir.exists() else []
        missing, invalid, instances = 0, 0, 0
        classes = {}
        for image_path in images:
            relative=image_path.relative_to(dataset_root/'images')
            label_path = (dataset_root/'labels'/relative).with_suffix('.txt')
            if not label_path.exists():
                missing += 1
                continue
            image = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                issues.append(f"无法读取图片: {image_path}")
                continue
            for line_no, line in enumerate(label_path.read_text(encoding="utf-8").splitlines(), 1):
                parts = line.split()
                if len(parts) != 5:
                    issues.append(f"{label_path}:{line_no} 需要 5 列")
                    invalid += 1
                    continue
                try:
                    cls = int(parts[0])
                    vals = [float(x) for x in parts[1:]]
                except ValueError:
                    issues.append(f"{label_path}:{line_no} 含非数字")
                    invalid += 1
                    continue
                if cls < 0 or cls >= nc or not np.isfinite(vals).all() or any(v < 0 or v > 1 for v in vals) or vals[2] <= 0 or vals[3] <= 0 or vals[0]-vals[2]/2 < -1e-6 or vals[0]+vals[2]/2 > 1+1e-6 or vals[1]-vals[3]/2 < -1e-6 or vals[1]+vals[3]/2 > 1+1e-6:
                    issues.append(f"{label_path}:{line_no} 类别或归一化坐标越界")
                    invalid += 1
                classes[str(cls)] = classes.get(str(cls), 0) + 1
                instances += 1
        splits[split] = {"image_count": len(images), "instance_count": instances, "missing_labels": missing, "invalid_lines": invalid, "classes": classes}
        if missing:issues.append(f'{split} 缺少 {missing} 个标签文件')
        if not images:
            issues.append(f"{split} 没有图片: {image_dir}")
    result = {"data": str(args.data), "nc": nc, "names": names, "splits": splits, "issues": issues, "ok": not issues}
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    print(text, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
