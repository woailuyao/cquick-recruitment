# -*- coding: utf-8 -*-
"""标注质量抽查（细则 1.5）：随机抽 30 张把框画回原图，并检查标签合法性。

运行:
    python verify_labels.py --data-root soccer_dataset --n 30

检查:
  - 每张图是否有对应的 .txt 标签
  - 每行是否为 5 个数、class_id 是否恒为 0
  - 归一化坐标是否越界、宽高是否 > 0
  - 同一段视频的帧是否被分到多个集合（数据泄漏，会被扣分）
输出: label_check/ 下的可视化图，供自己核对，不需要提交 GitHub。
"""
import argparse
import csv
import random
from collections import defaultdict
from pathlib import Path

import cv2

IMG_EXT = {".jpg", ".jpeg", ".png"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default="soccer_dataset")
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--out", default="label_check")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    root = Path(a.data_root)
    imgs = sorted(p for p in (root / "images").glob("*/*")
                  if p.suffix.lower() in IMG_EXT)
    if not imgs:
        raise SystemExit("[FAIL] 没有图片，等抽帧/标注后再运行")
    picked = random.Random(a.seed).sample(imgs, min(a.n, len(imgs)))

    outdir = Path(a.out)
    outdir.mkdir(parents=True, exist_ok=True)
    problems, n_boxes = [], 0
    for img_path in picked:
        split = img_path.parent.name
        label_path = root / "labels" / split / (img_path.stem + ".txt")
        frame = cv2.imread(str(img_path))
        if frame is None:
            problems.append("{}: 图片读不出来".format(img_path.name))
            continue
        h, w = frame.shape[:2]
        if not label_path.exists():
            problems.append("{}: 缺标签 {}".format(img_path.name, label_path.name))
        else:
            for ln, line in enumerate(label_path.read_text(encoding="utf-8").splitlines(), 1):
                parts = line.strip().split()
                if len(parts) != 5:
                    problems.append("{} 第{}行: 不是 5 个数 -> {}".format(
                        label_path.name, ln, line))
                    continue
                try:
                    cid, x, y, bw, bh = [float(p) for p in parts]
                except ValueError:
                    problems.append("{} 第{}行: 非数字".format(label_path.name, ln))
                    continue
                if int(cid) != 0:
                    problems.append("{} 第{}行: class_id={} 必须为 0(ball)".format(
                        label_path.name, ln, int(cid)))
                if not (0 <= x <= 1 and 0 <= y <= 1 and 0 < bw <= 1 and 0 < bh <= 1):
                    problems.append("{} 第{}行: 坐标越界或宽高<=0".format(label_path.name, ln))
                if x - bw / 2 < 0 or x + bw / 2 > 1 or y - bh / 2 < 0 or y + bh / 2 > 1:
                    problems.append("{} 第{}行: 框超出图像边界".format(label_path.name, ln))
                x1, y1 = int((x - bw / 2) * w), int((y - bh / 2) * h)
                x2, y2 = int((x + bw / 2) * w), int((y + bh / 2) * h)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, "ball", (x1, max(y1 - 5, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                n_boxes += 1
        cv2.imwrite(str(outdir / img_path.name), frame)

    manifest = root / "split_manifest.csv"
    if manifest.exists():
        seen = defaultdict(set)
        with manifest.open(newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                seen[row["source_video"]].add(row["split"])
        for video, splits in seen.items():
            if len(splits) > 1:
                problems.append("数据泄漏: {} 同时出现在 {}".format(video, sorted(splits)))

    print("抽查 {} 张 / 共 {} 张，画出 {} 个框，可视化输出到 {}/".format(
        len(picked), len(imgs), n_boxes, outdir))
    if problems:
        print("发现 {} 个问题:".format(len(problems)))
        for p in problems[:30]:
            print("  -", p)
        return 1
    print("标签格式、类别编号、坐标范围、集合划分检查 -> PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
