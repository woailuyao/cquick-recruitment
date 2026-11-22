# -*- coding: utf-8 -*-
"""
数据集整理 + 训练 + 按类别评估

三件事一起做，因为它们强相关：

  1. **按独立视频片段划分** train/val/test
     细则要求："不能先把一段视频的相邻帧随机打散到三个集合，否则评估结果可能虚高"
     所以这里以「片段文件夹」为最小单位整体分配，同一段视频的所有帧必然落在同一个集合。

  2. 训练（默认 YOLOv8n，CPU）
     本机 8 GB 内存，必须 workers=0、batch 小；详见 --help。

  3. 按类别分别报 Precision / Recall / mAP50 / mAP50-95
     细则要求任务 1 和任务 2 分别出指标，所以即使是同一份数据也要分类别看。

用法：

    # 只整理数据集，先看看划分结果
    python scripts/train.py --src dataset_raw --task ball --prepare-only

    # 整理 + 训练
    python scripts/train.py --src dataset_raw --task ball \
        --epochs 50 --imgsz 640 --batch 8 --workers 0 --device cpu

    # 场地标志（4 类）
    python scripts/train.py --src dataset_marker --task marker --epochs 50

产物：
    datasets/ball/
      ├── images/{train,val,test}/
      ├── labels/{train,val,test}/
      ├── data.yaml
      └── split_info.json        # 哪些片段进了哪个集合，便于复核
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import random
import shutil
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

sys.path.insert(0, ROOT)
from src.config import FIELD_MARKER_CLASSES  # noqa: E402
MARKER_ORDER = FIELD_MARKER_CLASSES          # 现场实际存在的类别，见 src/config.py


def build_dataset(src, out, task, val_ratio, test_ratio, seed, clean=True,
                  only_labeled=False, fixed_split=None, neg_segs=None):
    segs = sorted([d for d in glob.glob(os.path.join(src, "*")) if os.path.isdir(d)])
    segs = [d for d in segs if glob.glob(os.path.join(d, "*.jpg"))
            or glob.glob(os.path.join(d, "*.png"))]
    if not segs:
        raise SystemExit(f"{src} 下没有片段文件夹")

    # ---- 硬负样本增强 ----
    # 把"没有有效标志、但有相似干扰物"的片段以**空标注**加入训练集。
    # 典型场景：中线穿过中圈形成的「直线 × 圆弧」交叉，模型会把它误判成 TCross。
    # 这些帧在 only_labeled 模式下会被剔除，但作为纯负样本加入后，
    # 模型才能学到"这种形状不是标志"。
    neg_dirs = []
    if neg_segs:
        keys = [k.strip() for k in neg_segs.split(",") if k.strip()]
        neg_dirs = [d for d in segs if any(k in os.path.basename(d) for k in keys)]
        cleared = []
        for d in neg_dirs:
            n_cleared = 0
            for p in glob.glob(os.path.join(d, "*.jpg")) + glob.glob(os.path.join(d, "*.png")):
                tp = os.path.splitext(p)[0] + ".txt"
                if os.path.isfile(tp) and os.path.getsize(tp) > 0:
                    open(tp, "w", encoding="utf-8").close()   # 清空，作为纯负样本
                    n_cleared += 1
            if n_cleared:
                cleared.append(f"{os.path.basename(d)}(清空 {n_cleared} 个标签)")
        print(f"[--neg-segs] 作为硬负样本加入 {len(neg_dirs)} 个片段: "
              f"{[os.path.basename(d) for d in neg_dirs]}")
        if cleared:
            print(f"[--neg-segs] 以下片段原有标注被清空: {cleared}")
        segs = [d for d in segs if d not in neg_dirs] + neg_dirs

    # only_labeled 模式下，先把"整段都没有标注"的片段剔除，再划分。
    # 否则这些片段可能被分到 val/test，导致那个集合一张图都没有，训练直接报错。
    # 实拍里很常见：某段视频只拍到了中圈圆弧，没有 L/T 标志，整段空标注。
    if only_labeled:
        # 负样本片段本来就是空的，必须保留，不能按"无标注"剔除
        neg_set = set(neg_dirs)
        def has_any_label(d):
            if d in neg_set:
                return True
            for p in glob.glob(os.path.join(d, "*.jpg")) + glob.glob(os.path.join(d, "*.png")):
                tp = os.path.splitext(p)[0] + ".txt"
                if os.path.isfile(tp) and os.path.getsize(tp) > 0:
                    return True
            return False

        dropped = [os.path.basename(d) for d in segs if not has_any_label(d)]
        segs = [d for d in segs if has_any_label(d)]
        if dropped:
            print(f"[--only-labeled] 剔除整段无标注的片段 {len(dropped)} 个: {dropped}")
        if not segs:
            raise SystemExit("所有片段都没有标注，无法训练。")

    names = [os.path.basename(d) for d in segs]
    rng = random.Random(seed)
    rng.shuffle(names)

    # 手动指定划分：--fixed-split "test=224416;val=224011;train=223918,224316"
    # 为什么要这个开关：6 个视频片段各对应一个独立视角（贴脸近景 / 广角远景），
    # 按比例随机划分很容易把唯一的远景片段分到 test，导致训练时模型从没见过小尺度标志，
    # 测试指标崩到 0.03。手动指定可以保证训练集里同时包含近景和远景。
    if fixed_split:
        split = {"train": [], "val": [], "test": []}
        used = set()
        for part in fixed_split.split(";"):
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            k = k.strip()
            if k not in split:
                continue
            keys = [t.strip() for t in v.split(",") if t.strip()]
            for name in names:
                if any(key in name for key in keys):
                    split[k].append(name)
                    used.add(name)
        # 没被指到的片段归入 train，避免丢数据
        leftover = [n for n in names if n not in used]
        split["train"].extend(leftover)
        if leftover:
            print(f"[--fixed-split] 未指定的片段归入 train: {leftover}")
        if not split["train"]:
            raise SystemExit("--fixed-split 之后训练集为空，检查参数")
        print(f"[--fixed-split] train={split['train']}")
        print(f"[--fixed-split] val  ={split['val']}")
        print(f"[--fixed-split] test ={split['test']}")
    else:
        n = len(names)
        # 注意：不能无条件 max(1, ...)，否则 test_ratio=0 时仍然会强行分出一个测试集。
        # 只有明确给了正的 ratio 才保证至少 1 个片段。
        n_test = max(1, round(n * test_ratio)) if (n >= 3 and test_ratio > 0) else 0
        n_val = max(1, round(n * val_ratio)) if (n >= 3 and val_ratio > 0) else 0
        if n_test + n_val >= n:
            n_test = 0
            n_val = max(1, n - 1) if n >= 2 else 0

        split = {
            "test": names[:n_test],
            "val": names[n_test:n_test + n_val],
            "train": names[n_test + n_val:],
        }

    if clean and os.path.isdir(out):
        shutil.rmtree(out)
    for s in ("train", "val", "test"):
        os.makedirs(os.path.join(out, "images", s), exist_ok=True)
        os.makedirs(os.path.join(out, "labels", s), exist_ok=True)

    stats = {}
    # 负样本片段里的图按"空标注"照常拷入 —— 它们是刻意构造的硬负样本，
    # 不能因为"没有标注"就被 only_labeled 过滤掉，否则负样本增强完全失效。
    neg_names = {os.path.basename(d) for d in neg_dirs}
    for s, seg_names in split.items():
        n_img = n_box = n_empty = n_skip = 0
        for seg in seg_names:
            d = os.path.join(src, seg)
            is_neg = seg in neg_names
            for p in sorted(glob.glob(os.path.join(d, "*.jpg"))
                            + glob.glob(os.path.join(d, "*.png"))):
                tp = os.path.splitext(p)[0] + ".txt"
                has_label = os.path.isfile(tp) and os.path.getsize(tp) > 0

                if is_neg:
                    has_label = False        # 强制当负样本

                # 只训练已标注的图。
                # 这个开关很关键：做"种子模型"时，大部分图还没标，
                # 如果把它们当成"没有目标"的负样本，模型会被训成
                # "什么都检测不到"，整个 bootstrap 流程就废了。
                # 例外：负样本片段必须照常拷入（它们本来就没有标注）。
                if only_labeled and not has_label and not is_neg:
                    n_skip += 1
                    continue

                stem = os.path.splitext(os.path.basename(p))[0]
                newname = f"{seg}__{stem}"
                shutil.copy2(p, os.path.join(out, "images", s, newname + os.path.splitext(p)[1]))
                dst_txt = os.path.join(out, "labels", s, newname + ".txt")
                if has_label:
                    with open(tp, "r", encoding="utf-8") as f:
                        lines = [ln for ln in f.read().splitlines() if ln.strip()]
                    n_box += len(lines)
                    with open(dst_txt, "w", encoding="utf-8") as f:
                        f.write("\n".join(lines) + ("\n" if lines else ""))
                else:
                    open(dst_txt, "w").close()
                    n_empty += 1
                n_img += 1
        stats[s] = {"segments": len(seg_names), "images": n_img,
                    "boxes": n_box, "empty": n_empty, "skipped_unlabeled": n_skip}

    names_yaml = ["ball"] if task == "ball" else MARKER_ORDER
    # 注意：**不要写绝对路径**。
    # 之前写成 os.path.abspath(out)，换一台电脑就必然失效（比如 D:\vision_ws\... 在别人机器上不存在）。
    # 不写 path 时，ultralytics 会以 data.yaml 所在目录为基准解析 train/val/test，
    # 这样整个 datasets/ball 目录可以直接搬到任何机器上用。
    data = {
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "nc": len(names_yaml),
        "names": names_yaml,
    }
    with open(os.path.join(out, "data.yaml"), "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    with open(os.path.join(out, "split_info.json"), "w", encoding="utf-8") as f:
        json.dump({"split": split, "stats": stats, "seed": seed}, f,
                  ensure_ascii=False, indent=2)

    print("=" * 66)
    print(f"数据集: {out}")
    print(f"类别  : {names_yaml}")
    print("=" * 66)
    print(f"{'集合':<8}{'片段':>6}{'图片':>8}{'框':>8}{'空标注':>8}")
    print("-" * 66)
    for s in ("train", "val", "test"):
        st = stats[s]
        print(f"{s:<8}{st['segments']:>6}{st['images']:>8}{st['boxes']:>8}{st['empty']:>8}")
    print("-" * 66)
    print("划分明细（同一段视频整体进同一个集合，杜绝相邻帧泄漏）:")
    for s in ("train", "val", "test"):
        print(f"  {s:<6} {split[s]}")
    return os.path.join(out, "data.yaml"), out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(ROOT, "dataset_raw"))
    ap.add_argument("--out", default=None, help="默认 datasets/<task>")
    ap.add_argument("--task", choices=["ball", "marker"], default="ball")
    ap.add_argument("--val-ratio", type=float, default=0.2)
    ap.add_argument("--test-ratio", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--keep-old", action="store_true", help="不删除已有输出目录")
    ap.add_argument("--fixed-split", default=None,
                    help='手动指定片段划分，例如 "test=224416;val=224011;train=223918,224316"')
    ap.add_argument("--neg-segs", default=None,
                    help="硬负样本片段（逗号分隔的关键词）。这些片段的标注会被清空，"
                         "以纯负样本加入训练集，用来压制误检。"
                         "典型用途：中线穿过中圈形成的「直线×圆弧」交叉会被误判成 TCross")
    ap.add_argument("--only-labeled", action="store_true",
                    help="只用已标注（txt 非空）的图。做种子模型时必须加，"
                         "否则未标注的图会被当成负样本，把模型训成『什么都检测不到』")

    ap.add_argument("--prepare-only", action="store_true")
    ap.add_argument("--model", default=os.path.join(ROOT, "weights", "yolov8n.pt"))
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--workers", type=int, default=0,
                    help="8GB 内存必须用 0，否则数据加载子进程会爆内存")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--name", default=None, help="训练输出名，默认 <task>")
    # 本机有 8 GB 内存，这两项默认收紧
    args = ap.parse_args()

    out = args.out or os.path.join(ROOT, "datasets", args.task)
    data_yaml, outdir = build_dataset(args.src, out, args.task,
                                      args.val_ratio, args.test_ratio,
                                      args.seed, clean=not args.keep_old,
                                      only_labeled=args.only_labeled,
                                      fixed_split=args.fixed_split,
                                      neg_segs=args.neg_segs)
    if args.prepare_only:
        print("\n只整理数据集，未训练。训练命令：")
        print(f"  .\\.venv\\Scripts\\python.exe scripts\\train.py --src "
              f"{os.path.relpath(args.src, ROOT)} --task {args.task} --epochs {args.epochs}")
        return 0

    from ultralytics import YOLO
    name = args.name or args.task
    print(f"\n开始训练: model={os.path.basename(args.model)} epochs={args.epochs} "
          f"imgsz={args.imgsz} batch={args.batch} workers={args.workers} device={args.device}")
    print("提示：CPU 训练 200~300 张 50 epoch 约 1.5~2.5 小时，期间别开太多程序\n")

    model = YOLO(args.model)
    model.train(
        data=data_yaml, epochs=args.epochs, imgsz=args.imgsz,
        batch=args.batch, workers=args.workers, device=args.device,
        project=os.path.join(ROOT, "runs"), name=name, exist_ok=True,
        plots=False, val=True, verbose=True, seed=args.seed,
    )

    # ---- 在独立测试集上按类别评估 ----
    best = os.path.join(ROOT, "runs", name, "weights", "best.pt")
    if not os.path.isfile(best):
        print("[警告] 没找到 best.pt，跳过测试集评估")
        return 0

    # 如果没有测试集（例如 --test-ratio 0 做种子模型），跳过评估，
    # 否则 ultralytics 会因为 test 目录为空而抛 FileNotFoundError。
    test_dir = os.path.join(outdir, "images", "test")
    if not glob.glob(os.path.join(test_dir, "*")):
        print("\n[跳过测试集评估] 本次没有划分 test 集（通常是种子模型，只需要能用来预标注）")
        print(f"最佳权重: {best}")
        return 0

    print("\n" + "=" * 66)
    print("在独立测试集上评估（细则要求分别报告各项指标）")
    print("=" * 66)
    m = YOLO(best)
    metrics = m.val(data=data_yaml, split="test", imgsz=args.imgsz,
                    device=args.device, verbose=False, plots=False)

    names = metrics.names
    lines = []

    def row(label, p, r, ap50, ap):
        s = f"{label:<16}{p:>9.4f}{r:>9.4f}{ap50:>9.4f}{ap:>10.4f}"
        lines.append(s)
        print(s)

    print(f"{'class':<16}{'Precision':>9}{'Recall':>9}{'mAP50':>9}{'mAP50-95':>10}")
    print("-" * 66)
    box = metrics.box
    try:
        per_class = box.ap_class_index  # 类别索引列表
    except Exception:
        per_class = list(range(len(names)))
    if box.p is not None:
        # 总体
        row("all", float(box.mp), float(box.mr), float(box.map50), float(box.map))
        # 分类别
        for i, ci in enumerate(per_class):
            p = box.p[i] if i < len(box.p) else float("nan")
            r = box.r[i] if i < len(box.r) else float("nan")
            a50 = box.ap50[i] if i < len(box.ap50) else float("nan")
            a = box.ap[i] if i < len(box.ap) else float("nan")
            row(names.get(ci, str(ci)), float(p), float(r), float(a50), float(a))
    print("-" * 66)

    rep = os.path.join(ROOT, "runs", name, "test_metrics.txt")
    with open(rep, "w", encoding="utf-8") as f:
        f.write("class\tPrecision\tRecall\tmAP50\tmAP50-95\n")
        for ln in lines:
            parts = ln.split()
            f.write("\t".join(parts) + "\n")
    print(f"指标已写入 {rep}")
    print(f"最佳权重: {best}")
    print("\n提醒：把上面这段指标贴进 README 的『Precision / Recall / mAP50 / mAP50-95』，")
    print("      并且至少准备一个误检案例和一个漏检案例的说明。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
