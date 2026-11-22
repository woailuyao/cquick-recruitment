# -*- coding: utf-8 -*-
"""按视频段划分场地标志数据集（避免相邻帧泄漏）。

与 split_dataset.py 的区别：标志帧按段分在子目录里（dataset/raw_marker/marker_s0X/），
且不同段的文件名会重名（frame_0001.png），所以复制时统一改名为 `<段>__<原名>`。

没有 .txt 的图片视为「未标注」直接排除；.txt 为空视为「明确的负样本」保留。

用法（项目根目录）：
    .\\run.bat tools\\split_marker_dataset.py
    .\\run.bat tools\\split_marker_dataset.py --assign "s03,s01:s04:s02,s05"
"""

from __future__ import annotations

import argparse
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.common.paths import ROOT, rel          # noqa: E402

CLASSES = ["LCross", "TCross", "XCross"]   # 点球点样本已剔除（79 个误标为白线），暂不训练


def read_cls(txt: Path) -> list[int]:
    out = []
    if not txt.exists():
        return out
    for line in txt.read_text(encoding="utf-8").splitlines():
        p = line.split()
        if len(p) >= 5:
            try:
                out.append(int(float(p[0])))
            except ValueError:
                pass
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="按段划分标志数据集")
    ap.add_argument("--src", default="dataset/raw_marker")
    ap.add_argument("--out", default="dataset_marker")
    ap.add_argument("--assign", default="s03,s01:s04:s02,s05",
                    help="train:val:test，逗号分隔段名后缀，用冒号分三组")
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--mode", choices=["segment", "frame"], default="segment",
                    help="segment=按段整体划分（无泄漏但跨场景不泛化）；"
                         "frame=段内按时间划分（三份都含全部场景，但同场景有泄漏风险）")
    ap.add_argument("--gap", type=int, default=8,
                    help="frame 模式下两份之间空出的帧数，减少相邻帧近似重复")
    args = ap.parse_args()

    src = Path(args.src)
    if not src.is_absolute():
        src = ROOT / src
    out = Path(args.out)
    if not out.is_absolute():
        out = ROOT / out

    groups = {}
    for name, spec in zip(("train", "val", "test"), args.assign.split(":")):
        groups[name] = [t.strip() for t in spec.split(",") if t.strip()]

    imgs = sorted(src.rglob("*.png"))
    if not imgs:
        print(f"{rel(src)} 里没有图片")
        return 1

    by_seg = defaultdict(list)
    for p in imgs:
        if not p.with_suffix(".txt").exists():
            continue                      # 未标注，排除
        key = p.parent.name.replace("marker_", "")
        by_seg[key].append(p)

    if args.clean:
        for sub in ("images", "labels"):
            d = out / sub
            if d.exists():
                shutil.rmtree(d)

    print("=" * 74)
    print("段 -> 划分：")
    seg2split = {}
    for split, keys in groups.items():
        for k in keys:
            seg2split[k] = split
    for k in sorted(by_seg):
        print(f"  marker_{k:<6} {len(by_seg[k]):4d} 张 -> {seg2split.get(k, '（未分配，跳过）')}")
    print("=" * 74)

    summary = {}
    assignments = {"train": [], "val": [], "test": []}
    if args.mode == "frame":
        # 段内按时间切：每段前 65% 训、68–80% 验、83% 之后测，
        # 三份之间各空出几帧（由上面比例自带），减少相邻帧近似重复。
        for k, paths in by_seg.items():
            n = len(paths)
            a, b, c = int(n * 0.65), int(n * 0.68), int(n * 0.80)
            d = int(n * 0.83)
            for i, p in enumerate(paths):
                if i < a:
                    assignments["train"].append((k, p))
                elif b <= i < c:
                    assignments["val"].append((k, p))
                elif i >= d:
                    assignments["test"].append((k, p))
        kept = sum(len(v) for v in assignments.values())
        print(f"段内时间划分：保留 {kept} / {len(imgs)} 帧（三段之间各留缓冲帧）")
    else:
        for k, paths in by_seg.items():
            sp = seg2split.get(k)
            if sp in assignments:
                assignments[sp].extend((k, p) for p in paths)

    for split in ("train", "val", "test"):
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)
        n_img = n_box = 0
        cls = Counter()
        for k, p in assignments[split]:
            newname = f"marker_{k}__{p.stem}"
            shutil.copy2(p, out / "images" / split / f"{newname}.png")
            lbl = read_cls(p.with_suffix(".txt"))
            (out / "labels" / split / f"{newname}.txt").write_text(
                p.with_suffix(".txt").read_text(encoding="utf-8"), encoding="utf-8")
            n_img += 1
            n_box += len(lbl)
            cls.update(lbl)
        summary[split] = dict(img=n_img, box=n_box, cls=cls)
        dist = ", ".join(f"{CLASSES[c]}={cls.get(c, 0)}" for c in range(len(CLASSES)))
        print(f"{split:<6} 图片 {n_img:4d}  框 {n_box:5d}   {dist}")

    total_box = sum(v["box"] for v in summary.values())
    yml = out / "marker_data.yaml"
    # ultralytics 解析相对 path 时以「工作目录」为首选；项目根就是 week3-4，
    # 所以这里必须写相对项目根的路径（如 dataset_marker），不能写 Week3-4/dataset_marker。
    try:
        rel_path = out.resolve().relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        rel_path = str(out.resolve()).replace("\\", "/")
    yml.write_text(
        f"path: {rel_path}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"test: images/test\n"
        f"nc: {len(CLASSES)}\n"
        f"names: {CLASSES}\n",
        encoding="utf-8")

    lines = ["# 场地标志数据集统计", "",
             "## 划分（按视频段整体划分，避免相邻帧泄漏）", "",
             "| 划分 | 视频段 | 图片 | 框 | 类别分布 |", "| --- | --- | ---: | ---: | --- |"]
    for split in ("train", "val", "test"):
        segs = ",".join(f"marker_{k}" for k, v in seg2split.items() if v == split)
        v = summary[split]
        dist = " ".join(f"{CLASSES[c]}={v['cls'].get(c, 0)}" for c in range(len(CLASSES)))
        lines.append(f"| {split} | {segs} | {v['img']} | {v['box']} | {dist} |")
    lines += ["", f"合计 {total_box} 个框。", "",
              "## 说明", "",
              "- 类别顺序：`LCross=0, TCross=1, XCross=2, PenaltyPoint=3`。",
              "- 未标注（无 .txt）的图片已排除；空 .txt 视为明确的负样本保留。",
              "- 各段拍摄对象不同（s02 近景中圈交点、s04 遮挡、s05 另一光照），",
              "  因此单段内类别较单一，划分时已尽量让 train/val/test 各含多类。"]
    (out / "dataset_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("-" * 74)
    print(f"data.yaml: {rel(yml)}")
    print(f"统计: {rel(out / 'dataset_summary.md')}")
    print(f"合计 {total_box} 个框")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
