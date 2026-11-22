#!/usr/bin/env python3
"""Render a deterministic 30-image YOLO annotation QA contact sheet."""
import argparse
import random
from pathlib import Path

import cv2
import numpy as np


NAMES = ["LCross", "TCross", "XCross", "PenaltyPoint"]
COLORS = [(0, 255, 255), (0, 255, 0), (255, 0, 255), (0, 165, 255)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", default="frames")
    ap.add_argument("--output", default="label_qa_30_20261118.jpg")
    ap.add_argument("--seed", type=int, default=20261118)
    ap.add_argument("--count", type=int, default=30)
    args = ap.parse_args()
    root = Path(args.frames)
    labels = root / "labels"
    # Only images with a label file are active. The 22 video is deliberately
    # excluded from this L/T/X-only round by leaving its labels absent.
    active = [p for p in sorted(root.glob("*.jpg"))
              if (labels / f"{p.stem}.txt").exists()]
    rng = random.Random(args.seed)
    selected = rng.sample(active, min(args.count, len(active)))
    tiles = []
    for image_path in selected:
        image = cv2.imread(str(image_path))
        h, w = image.shape[:2]
        label_path = labels / f"{image_path.stem}.txt"
        lines = [x.strip() for x in label_path.read_text(encoding="utf-8-sig").splitlines()
                 if x.strip()]
        for line in lines:
            cls, cx, cy, bw, bh = line.split()
            cls = int(cls); cx = float(cx); cy = float(cy); bw = float(bw); bh = float(bh)
            x1 = int(round((cx-bw/2)*w)); y1 = int(round((cy-bh/2)*h))
            x2 = int(round((cx+bw/2)*w)); y2 = int(round((cy+bh/2)*h))
            cv2.rectangle(image, (x1, y1), (x2, y2), COLORS[cls], 4)
            cv2.putText(image, NAMES[cls], (max(2, x1), max(24, y1-7)),
                        cv2.FONT_HERSHEY_SIMPLEX, .8, COLORS[cls], 2, cv2.LINE_AA)
        tile = cv2.resize(image, (384, 216))
        caption = f"{image_path.name} | boxes={len(lines)}"
        cv2.rectangle(tile, (0, 188), (384, 216), (0, 0, 0), -1)
        cv2.putText(tile, caption, (5, 208), cv2.FONT_HERSHEY_SIMPLEX,
                    .42, (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(tile)
    while len(tiles) < args.count:
        tiles.append(np.zeros((216, 384, 3), np.uint8))
    rows = [np.hstack(tiles[i:i+5]) for i in range(0, args.count, 5)]
    sheet = np.vstack(rows)
    cv2.imwrite(args.output, sheet)
    Path(args.output).with_suffix(".txt").write_text(
        "\n".join(p.name for p in selected)+"\n", encoding="utf-8")
    print(f"active={len(active)} selected={len(selected)} output={args.output}")


if __name__ == "__main__":
    main()
