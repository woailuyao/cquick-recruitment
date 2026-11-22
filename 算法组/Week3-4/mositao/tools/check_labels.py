# -*- coding: utf-8 -*-
"""标注质检：细则 1.5 要求「随机抽查 30 张，将标签框画回原图」并检查若干缺陷。

检查项：
    1. 每张图是否有对应标签（缺失则补空文件，作为负样本）
    2. 每行格式：5 个字段、class=0、归一化坐标在 [0,1]、w/h > 0
    3. 框是否越界（换算像素后超出图像）
    4. 异常尺寸：极小框（<1e-4 面积）与极大框（>80% 画面）
    5. 跨视频段的完全重复图像（会导致数据泄漏）
    6. 各类统计：每段图片数、框数、空图数、框面积分布

产出：
    outputs/qc/qc_XX.png      30 张抽检图（框已画回原图）
    outputs/qc/report.txt     质检报告

用法：
    python tools/check_labels.py
    python tools/check_labels.py --samples 30 --fix-empty
"""

from __future__ import annotations

import argparse
import hashlib
import random
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import OUTPUT_DIR, rel                      # noqa: E402
from tools.annotate import load_boxes, yolo_to_xyxy              # noqa: E402

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                                  # noqa: BLE001
    pass


def md5_of(path: Path) -> str:
    h = hashlib.md5()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description="标注质检")
    ap.add_argument("--dir", default="dataset/pre")
    ap.add_argument("--samples", type=int, default=30)
    ap.add_argument("--fix-empty", action="store_true",
                    help="为没有标签文件的图补一个空 txt（负样本）")
    args = ap.parse_args()

    d = Path(args.dir)
    if not d.is_absolute():
        d = ROOT / d
    qc_dir = OUTPUT_DIR / "qc"
    qc_dir.mkdir(parents=True, exist_ok=True)

    imgs = sorted(d.glob("*.png"))
    if not imgs:
        print(f"{rel(d)} 里没有图片")
        return 1

    problems: list[str] = []
    warns: list[str] = []
    per_seg = defaultdict(lambda: {"img": 0, "box": 0, "empty": 0})
    seg_owner: dict[str, set] = defaultdict(set)     # 内容指纹 -> 段集合
    areas_all: list[float] = []

    missing = []
    for img in imgs:
        seg = img.stem.split("__")[0]
        per_seg[seg]["img"] += 1
        txt = img.with_suffix(".txt")
        if not txt.exists():
            missing.append(img)
            if args.fix_empty:
                txt.write_text("", encoding="utf-8")

        boxes = load_boxes(txt)
        per_seg[seg]["box"] += len(boxes)
        if not boxes:
            per_seg[seg]["empty"] += 1

        # 读图尺寸用于越界判定
        arr = cv2.imread(str(img))
        if arr is None:
            problems.append(f"{img.name}: 图像无法读取")
            continue
        h, w = arr.shape[:2]

        for k, b in enumerate(boxes):
            cls, cx, cy, bw, bh = b
            if int(cls) != 0:
                problems.append(f"{img.name} 第{k + 1}行: class={int(cls)} 不是 0(ball)")
            if not (0 <= cx <= 1 and 0 <= cy <= 1):
                problems.append(f"{img.name} 第{k + 1}行: 中心点越界 ({cx:.4f},{cy:.4f})")
            if bw <= 0 or bh <= 0:
                problems.append(f"{img.name} 第{k + 1}行: 宽高非正 ({bw:.4f},{bh:.4f})")
            x1, y1, x2, y2 = yolo_to_xyxy(b, w, h)
            if x1 < -1 or y1 < -1 or x2 > w + 1 or y2 > h + 1:
                warns.append(f"{img.name} 第{k + 1}行: 框超出图像边界")
            area = bw * bh
            areas_all.append(area)
            if area < 1e-4:
                warns.append(f"{img.name} 第{k + 1}行: 极小框 面积占比 {area:.2e}")
            if area > 0.8:
                warns.append(f"{img.name} 第{k + 1}行: 极大框 面积占比 {area:.2f}")

        seg_owner[md5_of(img)].add(seg)

    dup_across = [k for k, segs in seg_owner.items() if len(segs) > 1]
    if dup_across:
        problems.append(f"发现 {len(dup_across)} 组跨视频段的完全重复图像（数据泄漏风险）")

    # 抽检图
    random.seed(20261110)
    picks = random.sample(imgs, min(args.samples, len(imgs)))
    for i, img in enumerate(picks, 1):
        arr = cv2.imread(str(img))
        if arr is None:
            continue
        boxes = load_boxes(img.with_suffix(".txt"))
        h, w = arr.shape[:2]
        for b in boxes:
            x1, y1, x2, y2 = yolo_to_xyxy(b, w, h)
            cv2.rectangle(arr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 220, 0), 2)
        cv2.putText(arr, f"{img.name}  boxes={len(boxes)}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imwrite(str(qc_dir / f"qc_{i:02d}.png"), arr)

    # 报告
    total_img = len(imgs)
    total_box = sum(v["box"] for v in per_seg.values())
    total_empty = sum(v["empty"] for v in per_seg.values())
    lines = [
        "标注质检报告",
        "=" * 60,
        f"目录     : {rel(d)}",
        f"图片总数 : {total_img}",
        f"框总数   : {total_box}",
        f"空标签图 : {total_empty}（负样本，属正常）",
        f"缺标签文件: {len(missing)}" + ("（已补空文件）" if args.fix_empty else ""),
        "",
        "每个视频段：",
    ]
    for seg in sorted(per_seg):
        v = per_seg[seg]
        lines.append(f"  {seg:<12} 图片 {v['img']:3d}  框 {v['box']:3d}  空图 {v['empty']:2d}")
    if areas_all:
        a = np.array(areas_all)
        lines += ["", "框面积占画面比例：",
                  f"  最小 {a.min():.4f}  中位 {np.median(a):.4f}  最大 {a.max():.4f}",
                  f"  <0.001 的框 {int((a < 0.001).sum())} 个（远处小目标）"]
    lines += ["", f"错误 {len(problems)} 条，告警 {len(warns)} 条"]
    for p in problems[:40]:
        lines.append("  [错误] " + p)
    for w_ in warns[:40]:
        lines.append("  [告警] " + w_)
    if len(warns) > 40:
        lines.append(f"  ... 另有 {len(warns) - 40} 条告警")
    lines += ["", f"抽检图：{rel(qc_dir)}/qc_XX.png（{len(picks)} 张，框已画回原图）",
              "=" * 60]
    report = "\n".join(lines)
    (qc_dir / "report.txt").write_text(report, encoding="utf-8")
    print(report)
    print(f"\n报告已存：{rel(qc_dir / 'report.txt')}")
    return 0 if not problems else 1


if __name__ == "__main__":
    raise SystemExit(main())
