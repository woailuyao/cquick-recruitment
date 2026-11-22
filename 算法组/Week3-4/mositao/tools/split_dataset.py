# -*- coding: utf-8 -*-
"""按视频段划分数据集（细则 1.4：不能用相邻帧随机打散）。

要点：
    划分单位是**视频段**，不是图片——否则同一段视频的相邻帧会同时出现在训练集和验证集，
    指标虚高。段数只有 5 个时，70/20/10 无法精确，这里用穷举找到最接近的分配。

产出：
    dataset/images/{train,val,test}/*.png
    dataset/labels/{train,val,test}/*.txt
    dataset/data.yaml
    dataset/dataset_summary.md

用法：
    python tools/split_dataset.py
    python tools/split_dataset.py --ratios 0.7 0.2 0.1
"""

from __future__ import annotations

import argparse
import itertools
import shutil
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import save_yaml        # noqa: E402
from src.common.paths import DATASET_DIR, rel     # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="按视频段划分数据集")
    ap.add_argument("--src", default="dataset/pre")
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--ratios", nargs=3, type=float, default=[0.7, 0.2, 0.1])
    ap.add_argument("--clean", action="store_true", help="先清空已有的 images/labels 划分")
    ap.add_argument("--min-test", type=int, default=0,
                    help="测试集最少图片数；太小（如 10%% 以下）会让指标过于乐观")
    args = ap.parse_args()

    src = Path(args.src)
    if not src.is_absolute():
        src = ROOT / src
    out = Path(args.out)
    if not out.is_absolute():
        out = ROOT / out

    imgs = sorted(src.glob("*.png"))
    if not imgs:
        print(f"{rel(src)} 里没有图片")
        return 1

    by_seg: dict[str, list[Path]] = defaultdict(list)
    for p in imgs:
        by_seg[p.stem.split("__")[0]].append(p)
    segs = sorted(by_seg)
    total = len(imgs)
    print("=" * 70)
    print(f"来源 {rel(src)}：{total} 张，{len(segs)} 个视频段")
    for s in segs:
        print(f"  {s:<14} {len(by_seg[s]):4d} 张")
    print("=" * 70)

    if len(segs) < 3:
        print("视频段少于 3 个，无法划分三份")
        return 1

    # 穷举分配，近似目标比例（按图片数）
    target = [r * total for r in args.ratios]
    best = None
    for assign in itertools.product(range(3), repeat=len(segs)):
        if len(set(assign)) < 3:
            continue
        counts = [0, 0, 0]
        for s, k in zip(segs, assign):
            counts[k] += len(by_seg[s])
        if counts[2] < args.min_test:
            continue
        dev = sum(abs(counts[i] - target[i]) for i in range(3))
        if best is None or dev < best[0]:
            best = (dev, assign, counts)
    if best is None:
        print(f"在 --min-test {args.min_test} 约束下找不到有效划分，请放宽约束")
        return 1
    assert best is not None
    dev, assign, counts = best

    names = ["train", "val", "test"]
    print("划分方案（按段整体划分，避免帧泄漏）：")
    for s, k in zip(segs, assign):
        print(f"  {s:<14} -> {names[k]:<5} ({len(by_seg[s])} 张)")
    print(f"  图片数 train/val/test = {counts[0]}/{counts[1]}/{counts[2]} "
          f"（{counts[0] / total:.0%}/{counts[1] / total:.0%}/{counts[2] / total:.0%}，"
          f"目标 {args.ratios[0]:.0%}/{args.ratios[1]:.0%}/{args.ratios[2]:.0%}）")

    if args.clean:
        for sub in ("images", "labels"):
            target_dir = out / sub
            if target_dir.exists():
                shutil.rmtree(target_dir)
                print(f"已清空 {rel(target_dir)}")

    stats = {}
    for name in names:
        (out / "images" / name).mkdir(parents=True, exist_ok=True)
        (out / "labels" / name).mkdir(parents=True, exist_ok=True)

    for s, k in zip(segs, assign):
        name = names[k]
        stats[name] = {"images": 0, "boxes": 0, "segments": []}
        for img in by_seg[s]:
            shutil.copy2(img, out / "images" / name / img.name)
            txt = img.with_suffix(".txt")
            if txt.exists():
                shutil.copy2(txt, out / "labels" / name / txt.name)
            else:
                (out / "labels" / name / txt.name).write_text("", encoding="utf-8")
            stats[name]["images"] += 1
            nb = 0
            lf = img.with_suffix(".txt")
            if lf.exists():
                nb = sum(1 for ln in lf.read_text(encoding="utf-8").splitlines() if ln.strip())
            stats[name]["boxes"] += nb
        stats[name]["segments"].append(s)

    save_yaml(out / "data.yaml", {
        "path": str(Path(args.out)).replace("\\", "/"),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "nc": 1,
        "names": ["ball"],
    })

    md = [
        "# 任务一 数据集说明（dataset_summary）",
        "",
        f"- 原始视频段数：{len(segs)}",
        f"- 图片总数：{total}",
        f"- 类别：`ball`（单类别，class_id=0）",
        "- 划分单位：**视频段**（不是帧），因此同一段视频的相邻帧不会跨集合，无泄漏",
        "",
        "## 划分结果",
        "",
        "| 集合 | 视频段 | 图片数 | 标注框数 | 占比 |",
        "| --- | --- | ---: | ---: | ---: |",
    ]
    for name in names:
        st = stats[name]
        md.append(f"| {name} | {', '.join(st['segments'])} | {st['images']} | "
                  f"{st['boxes']} | {st['images'] / total:.0%} |")
    md += [
        "",
        "## 划分方法",
        "",
        "视频段数较少（5 段）时 70/20/10 无法精确命中，采用穷举所有段分配方案、",
        "取与目标比例图片数偏差最小者。这样既满足「按独立视频段划分」的硬要求，",
        "又让三份的比例尽量接近细则建议值。",
        "",
        "## 数据来源",
        "",
        "- 采集设备：配发的 USB 单目相机（Windows 索引 1，MSVID 0BDA:5844），1280x720@30fps",
        "- 采集方式：本人于实验室/场地实拍，共 5 段独立视频",
        "- 抽帧：按时间间隔采样 + 清晰度自适应阈值 + 相邻帧去重，避免高度相似帧",
        "- 预标注：YOLOv8n(COCO) 的 sports ball 类生成初始框，本人逐张人工修正",
        "",
        "## 已知局限",
        "",
        "- 数据以「草地 + 较大足球」的近景为主，远景小目标样本偏少",
        "- 场景光照以室内灯为主，强光/逆光样本有限",
    ]
    (out / "dataset_summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    print("-" * 70)
    print(f"已写出 {rel(out / 'data.yaml')} 与 {rel(out / 'dataset_summary.md')}")
    print(f"图片/标签已复制到 {rel(out / 'images')} 与 {rel(out / 'labels')}")
    print("下一步：python tools/train.py")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
