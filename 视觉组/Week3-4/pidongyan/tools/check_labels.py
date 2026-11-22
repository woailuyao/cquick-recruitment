"""标注质量检查：把标签框画回原图 + 数据合法性统计。

考核要求（1.5 节）训练前至少随机抽查 30 张，检查：
  漏标 / 错标 / 类别编号错误、框是否覆盖可见区域、框是否过大把背景框进去、
  标签坐标是否越界、宽高是否大于 0、train/val/test 是否存在相邻帧泄漏。

用法：
  # 标注阶段：检查 frames_pool 里刚标完的图
  python tools/check_labels.py --sample 30

  # 划分后：额外检查三集合是否有同源视频帧泄漏
  python tools/check_labels.py --images dataset/images/train --labels dataset/labels/train
  python tools/check_labels.py --dataset dataset          # 一次性检查三个集合 + 泄漏

输出：
  check_out/  抽查图（框已画回原图，人工逐张看）
  logs/label_check_<时间>.txt  统计报告
"""
import argparse
import datetime as dt
import random
from collections import Counter, defaultdict
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
SPLITS = ("train", "val", "test")


def group_key(stem: str) -> str:
    head, sep, tail = stem.rpartition("_f")
    return head if (sep and tail.isdigit()) else stem


def parse_label(path: Path, allowed_ids, tol=1e-3):
    """返回 (boxes, errors)。boxes = [(cls, cx, cy, w, h)]"""
    boxes, errors = [], []
    text = path.read_text(encoding="utf-8", errors="ignore").strip()
    if not text:
        return boxes, errors
    for ln, line in enumerate(text.splitlines(), 1):
        parts = line.split()
        if len(parts) != 5:
            errors.append(f"第{ln}行字段数={len(parts)}（应为5）")
            continue
        try:
            cls = int(float(parts[0]))
            cx, cy, w, h = (float(v) for v in parts[1:])
        except ValueError:
            errors.append(f"第{ln}行含非数值")
            continue
        if cls not in allowed_ids:
            errors.append(f"第{ln}行类别id={cls} 不在 {sorted(allowed_ids)}")
        if w <= 0 or h <= 0:
            errors.append(f"第{ln}行宽高<=0 ({w},{h})")
        if not (-tol <= cx <= 1 + tol and -tol <= cy <= 1 + tol):
            errors.append(f"第{ln}行中心越界 ({cx:.3f},{cy:.3f})")
        if not (0 < w <= 1 + tol and 0 < h <= 1 + tol):
            errors.append(f"第{ln}行宽高越界 ({w:.3f},{h:.3f})")
        if abs(cx - 0.5) * 2 + w / 2 > 1 + tol or abs(cy - 0.5) * 2 + h / 2 > 1 + tol:
            errors.append(f"第{ln}行框超出图像边界")
        boxes.append((cls, cx, cy, w, h))
    return boxes, errors


def inspect(images_dir: Path, labels_dir: Path, allowed_ids):
    stats = {
        "images": 0, "labeled": 0, "missing": 0, "empty": 0,
        "bad_files": 0, "boxes": 0, "bad_boxes": 0,
        "oversized": 0, "tiny": 0,
    }
    cls_count = Counter()
    areas = []
    problems = []
    pairs = []

    for img in sorted(images_dir.iterdir()):
        if img.suffix.lower() not in IMG_EXTS:
            continue
        stats["images"] += 1
        lbl = labels_dir / (img.stem + ".txt")
        if not lbl.exists():
            stats["missing"] += 1
            problems.append(f"[缺标签] {img.name}")
            continue
        boxes, errors = parse_label(lbl, allowed_ids)
        if errors:
            stats["bad_files"] += 1
            problems.extend(f"[{img.name}] {e}" for e in errors)
        if not boxes:
            stats["empty"] += 1
        else:
            stats["labeled"] += 1
        for cls, cx, cy, w, h in boxes:
            stats["boxes"] += 1
            cls_count[cls] += 1
            area = w * h
            areas.append(area)
            if area > 0.70:
                stats["oversized"] += 1
                problems.append(f"[框过大 {area:.2f}] {img.name}")
            if area < 0.0005:
                stats["tiny"] += 1
        pairs.append((img, boxes))

    return stats, cls_count, areas, problems, pairs


def draw(img_path: Path, boxes, out_path: Path, names):
    img = cv2.imread(str(img_path))
    if img is None:
        return False
    h, w = img.shape[:2]
    for cls, cx, cy, bw, bh in boxes:
        x1 = int((cx - bw / 2) * w)
        y1 = int((cy - bh / 2) * h)
        x2 = int((cx + bw / 2) * w)
        y2 = int((cy + bh / 2) * h)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 165, 255), 2)
        label = f"{names.get(cls, cls)} {int((x2 - x1))}x{int((y2 - y1))}"
        cv2.putText(img, label, (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.imwrite(str(out_path), img)
    return True


def check_leakage(dataset_root: Path):
    seen = defaultdict(set)
    for s in SPLITS:
        d = dataset_root / "images" / s
        if not d.exists():
            continue
        for img in d.iterdir():
            if img.suffix.lower() in IMG_EXTS:
                seen[group_key(img.stem)].add(s)
    leaked = {k: sorted(v) for k, v in seen.items() if len(v) > 1}
    return leaked


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default=str(ROOT / "frames_pool"))
    ap.add_argument("--labels", default=None)
    ap.add_argument("--sample", type=int, default=30, help="抽样画框张数")
    ap.add_argument("--out", default=str(ROOT / "check_out"))
    ap.add_argument("--class-id", type=int, nargs="*", default=[0], help="允许的类别编号")
    ap.add_argument("--names", default="ball", help="类别名，用逗号分隔，如 ball 或 LCross,TCross,XCross,PenaltyPoint")
    ap.add_argument("--dataset", default=None, help="划分后传 dataset 根目录，额外做泄漏检查")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    images_dir = Path(args.images)
    labels_dir = Path(args.labels) if args.labels else images_dir
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    name_list = [s.strip() for s in args.names.split(",") if s.strip()]
    names = {i: n for i, n in zip(args.class_id, name_list)}
    allowed = set(args.class_id)

    if not images_dir.exists():
        raise SystemExit(f"目录不存在：{images_dir}")

    stats, cls_count, areas, problems, pairs = inspect(images_dir, labels_dir, allowed)

    lines = []
    lines.append(f"检查目录：{images_dir}")
    lines.append(f"图片总数 {stats['images']} | 有标注 {stats['labeled']} | "
                 f"空标签 {stats['empty']} | 缺标签 {stats['missing']} | "
                 f"标签文件有错 {stats['bad_files']}")
    lines.append(f"标注框总数 {stats['boxes']} | 过大(>70%面积) {stats['oversized']} | "
                 f"过小(<0.05%面积) {stats['tiny']}")
    if areas:
        lines.append(f"框面积占比 min={min(areas):.4f} mean={sum(areas)/len(areas):.4f} max={max(areas):.4f}")
    if cls_count:
        lines.append("各类别框数：" + ", ".join(
            f"{names.get(k, k)}={v}" for k, v in sorted(cls_count.items())))
    if problems:
        lines.append(f"\n问题明细（前 40 条，共 {len(problems)}）：")
        lines.extend(problems[:40])
    else:
        lines.append("\n未发现格式问题")

    if args.dataset:
        leaked = check_leakage(Path(args.dataset))
        if leaked:
            lines.append(f"\n[泄漏] {len(leaked)} 个视频组跨 split：")
            for k, v in list(leaked.items())[:10]:
                lines.append(f"  {k} -> {v}")
        else:
            lines.append("\n[泄漏检查] 通过：没有视频组跨 train/val/test")

    report = "\n".join(lines)
    print(report)

    log_path = ROOT / "logs" / f"label_check_{dt.datetime.now():%Y%m%d_%H%M%S}.txt"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(report, encoding="utf-8")
    print(f"\n报告已保存：{log_path}")

    if pairs:
        random.Random(args.seed).shuffle(pairs)
        picked = pairs[:args.sample]
        saved = 0
        for img, boxes in picked:
            dst = out_dir / f"{img.stem}_check.jpg"
            if draw(img, boxes, dst, names):
                saved += 1
        print(f"已画框 {saved} 张 -> {out_dir}")
        print("请逐张目视检查：框是否贴住足球可见区域、有没有漏标的球、有没有把地面框进去")
    else:
        print("没有可抽查的样本")


if __name__ == "__main__":
    main()
