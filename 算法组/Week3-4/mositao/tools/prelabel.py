# -*- coding: utf-8 -*-
"""预标注：用 COCO 预训练模型生成初始框，把人工修正的工作量压到 1/3。

原理：
    YOLOv8n 的 COCO 类别里本来就有 `sports ball`（class_id = 32）。
    对足球场景，它通常能给出不错的初始框；我们把它转成考核要求的单类别 `ball`，
    人工只做「删误标 / 补漏标 / 微调框」三件事。

输出目录结构（便于直接丢给 LabelImg）：
    dataset/pre/<视频段>/<帧名>.png
    dataset/pre/<视频段>/<帧名>.txt      # YOLO: class cx cy w h（归一化）
    dataset/pre/classes.txt              # 只有一行：ball

用法：
    python tools/prelabel.py
    python tools/prelabel.py --conf 0.2 --max-boxes 3
"""

from __future__ import annotations

import argparse
import shutil
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import DATASET_DIR, WEIGHTS_DIR, rel        # noqa: E402

BALL_CLASS_ID_IN_COCO = 32      # sports ball
IMG_EXTS = {".png", ".jpg", ".jpeg"}


def main() -> int:
    ap = argparse.ArgumentParser(description="COCO sports ball 预标注")
    ap.add_argument("--src", default="dataset/raw")
    ap.add_argument("--out", default="dataset/pre")
    ap.add_argument("--weights", default="yolov8n.pt")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--max-boxes", type=int, default=3, help="每帧最多保留几个球框")
    ap.add_argument("--device", default="0")
    ap.add_argument("--batch", type=int, default=16)
    args = ap.parse_args()

    src = Path(args.src)
    if not src.is_absolute():
        src = ROOT / src
    out = Path(args.out)
    if not out.is_absolute():
        out = ROOT / out
    wpath = Path(args.weights)
    if not wpath.is_absolute():
        wpath = WEIGHTS_DIR / wpath

    if not src.exists():
        print(f"没找到 {rel(src)}，请先跑 tools/frame_extract.py")
        return 1
    if not wpath.is_file():
        print(f"没找到权重 {rel(wpath)}")
        return 1
    out.mkdir(parents=True, exist_ok=True)

    from ultralytics import YOLO
    model = YOLO(str(wpath))

    groups = sorted([d for d in src.iterdir() if d.is_dir()])
    if not groups:
        print(f"{rel(src)} 下没有按视频分好的子目录")
        return 1

    print("=" * 74)
    print(f"预标注：{len(groups)} 个视频段｜conf≥{args.conf}｜权重 {rel(wpath)}")
    print("=" * 74)

    stats = []
    for g in groups:
        imgs = sorted(p for p in g.iterdir() if p.suffix.lower() in IMG_EXTS)
        if not imgs:
            continue
        # 拍平到单目录，文件名带视频段前缀：
        # ① LabelImg 只需开一个目录；② 划分数据集时靠前缀按段拆分，避免帧泄漏
        n_box, n_empty, n_multi = 0, 0, 0

        for i in range(0, len(imgs), args.batch):
            chunk = imgs[i:i + args.batch]
            results = model.predict([str(p) for p in chunk], imgsz=640, conf=args.conf,
                                    classes=[BALL_CLASS_ID_IN_COCO], device=args.device,
                                    verbose=False)
            for path, res in zip(chunk, results):
                stem = f"{g.name}__{path.stem}"
                shutil.copy2(path, out / (stem + ".png"))
                boxes = res.boxes
                lines = []
                if boxes is not None and len(boxes) > 0:
                    xywhn = boxes.xywhn.cpu().numpy()
                    confs = boxes.conf.cpu().numpy()
                    order = np.argsort(-confs)[: args.max_boxes]
                    for k in order:
                        cx, cy, w, h = [float(v) for v in xywhn[k]]
                        cx, cy = min(max(cx, 0.0), 1.0), min(max(cy, 0.0), 1.0)
                        w, h = min(max(w, 1e-6), 1.0), min(max(h, 1e-6), 1.0)
                        lines.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
                (out / (stem + ".txt")).write_text(
                    "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
                if not lines:
                    n_empty += 1
                else:
                    n_box += len(lines)
                    if len(lines) > 1:
                        n_multi += 1

        stats.append({"video": g.name, "images": len(imgs), "boxes": n_box,
                      "empty": n_empty, "multi": n_multi})
        print(f"{g.name:<28} 图片 {len(imgs):3d}  初始框 {n_box:3d}  "
              f"无框待补 {n_empty:3d}  多球帧 {n_multi:2d}")

    classes = out / "classes.txt"
    classes.write_text("ball\n", encoding="utf-8")

    tot_img = sum(s["images"] for s in stats)
    tot_box = sum(s["boxes"] for s in stats)
    tot_empty = sum(s["empty"] for s in stats)
    print("-" * 74)
    print(f"合计：图片 {tot_img} 张，初始框 {tot_box} 个，无框待补 {tot_empty} 张")
    print(f"标注目录：{rel(out)}")
    print(f"类别文件：{rel(classes)}（内容只有一行 ball）")
    print()
    print("人工修正要点（三件事，每张几秒）：")
    print("  1. 删除误标：白线、圆形物体被框成球的，删掉该行")
    print("  2. 补漏标：远处小球、被遮一半的球，按键新建框")
    print("  3. 微调框：框紧球可见区域，不要把大片地面框进来")
    print("修正完成后：python tools/check_labels.py（抽检 30 张并查泄漏）")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
