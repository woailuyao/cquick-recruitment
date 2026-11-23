"""任务二四类标志数据集划分：frames_pool_markers/ -> dataset_markers/。

与任务一 split_dataset.py 的区别（重要）
----------------------------------------
* 本脚本**只复制、只写不删**：不清空、不移动、不 unlink 任何已有文件（用户红线）。
* 打印口径是「含标志」而非「含球」，并给出四类各自的框数。
* 目标结构对应 config/marker.yaml 的 dataset_markers/。

按独立视频段划分：文件名 <视频名>_f<帧号>.jpg，同段帧不跨 split，防泄漏。
缺 .txt = 未标注（跳过，不会当成负样本）；
空 .txt = 合法负样本（本人在标注器里按 0 确认的「无标志帧」）。

用法（PowerShell 单行，整行复制粘贴）
--------------------------------------
  python tools/split_markers.py --dry-run
  python tools/split_markers.py
  python tools/split_markers.py --ratio 0.7 0.2 0.1 --seed 0
"""
import argparse
import random
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
SPLITS = ("train", "val", "test")
CLASSES = ("L_cross", "T_cross", "X_cross", "penalty_point")


def group_key(stem):
    """marker_run04b_20261112_165845_f000180 -> marker_run04b_20261112_165845"""
    head, sep, tail = stem.rpartition("_f")
    return head if (sep and tail.isdigit()) else stem


def collect(images_dir, labels_dir):
    groups = defaultdict(list)
    pos_count = defaultdict(int)          # 每组「含标志框」的图片数
    cls_count = Counter()                 # 四类框数
    missing, total = [], 0
    for img in sorted(images_dir.iterdir()):
        if img.suffix.lower() not in IMG_EXTS:
            continue
        total += 1
        lbl = labels_dir / (img.stem + ".txt")
        if not lbl.exists():
            missing.append(img.name)
            continue
        lines = [ln for ln in lbl.read_text(encoding="utf-8").splitlines() if ln.strip()]
        g = group_key(img.stem)
        groups[g].append((img, lbl))
        if lines:
            pos_count[g] += 1
        for ln in lines:
            try:
                cls_count[int(ln.split()[0])] += 1
            except (ValueError, IndexError):
                pass
    return groups, missing, pos_count, cls_count, total


def allocate(names, sizes, pos_count, ratio, seed):
    shuffled = list(names)
    random.Random(seed).shuffle(shuffled)
    pool = [n for n in shuffled if pos_count.get(n, 0) > 0]
    forced_train = [n for n in shuffled if pos_count.get(n, 0) == 0]
    plan = {s: [] for s in SPLITS}
    plan["train"].extend(forced_train)
    if forced_train:
        print(f"[负样本组] {len(forced_train)} 个纯空组强制进 train："
              f"{', '.join(forced_train)}")

    base = sum(sizes[n] for n in pool) or 1
    bounds = [ratio[0], ratio[0] + ratio[1]]
    cum = 0
    for n in pool:
        p = (cum + sizes[n] / 2) / base     # 用组中心判定归属
        if p < bounds[0]:
            plan["train"].append(n)
        elif p < bounds[1]:
            plan["val"].append(n)
        else:
            plan["test"].append(n)
        cum += sizes[n]

    # 保底：val / test 各至少 1 个含标志组，否则 P/R/mAP50 无法计算
    for s in ("val", "test"):
        if any(pos_count.get(n, 0) > 0 for n in plan[s]):
            continue
        donors = sorted((k for k in SPLITS if k != s), key=lambda k: -len(plan[k]))
        for d in donors:
            movable = [n for n in plan[d] if pos_count.get(n, 0) > 0]
            if movable and len(plan[d]) > 1:
                pick = movable[0]
                plan[d].remove(pick)
                plan[s].append(pick)
                print(f"[保底] 从 {d} 挪含标志组 {pick} 到 {s}")
                break
    return plan


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", default="frames_pool_markers")
    ap.add_argument("--labels", default=None, help="标签目录，默认与图片同目录")
    ap.add_argument("--out", default="dataset_markers")
    ap.add_argument("--ratio", nargs=3, type=float, default=[0.7, 0.2, 0.1],
                    metavar=("TRAIN", "VAL", "TEST"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dry-run", action="store_true", help="只看划分，不写文件")
    args = ap.parse_args()

    images_dir = (ROOT / args.images).resolve()
    labels_dir = (ROOT / args.labels).resolve() if args.labels else images_dir
    out_root = (ROOT / args.out).resolve()
    if abs(sum(args.ratio) - 1.0) > 1e-6:
        raise SystemExit("比例之和必须为 1")

    groups, missing, pos_count, cls_count, total = collect(images_dir, labels_dir)
    labeled = sum(len(v) for v in groups.values())
    print(f"素材 {total} 张 | 已标注 {labeled} 张 | 未标注 {len(missing)} 张 | "
          f"视频组 {len(groups)} 个（只统计已标注组）")

    if total and not groups:
        print("\n[还没开始标注] 先跑零安装标注器（只写 .txt，不动图片）：")
        print("  python tools/annotate_cv2.py --dir frames_pool_markers --classes L_cross,T_cross,X_cross,penalty_point")
        print("  左键拖拽画框 / 1~4 切类别 / 0 确认无标志 / n 下一张 / s 保存 / q 退出")
        print("标完再跑本脚本生成 dataset_markers/ 划分；--dry-run 可随时查看进度。")
        return 0
    if missing:
        print(f"[提示] {len(missing)} 张未标注，已跳过（不会当负样本混入）")
        if len(missing) <= 5:
            print("    " + ", ".join(missing))

    sizes = {n: len(v) for n, v in groups.items()}
    plan = allocate(list(groups.keys()), sizes, pos_count, args.ratio, args.seed)
    grand = sum(sizes.values()) or 1
    print("\n划分结果：")
    print(f"{'split':<8}{'组数':<6}{'图片':<6}{'含标志':<8}{'占比':<8}视频组")
    for s in SPLITS:
        cnt = sum(sizes[n] for n in plan[s])
        npos = sum(pos_count.get(n, 0) for n in plan[s])
        print(f"{s:<8}{len(plan[s]):<6}{cnt:<6}{npos:<8}{cnt/grand*100:>5.1f}%  "
              f"{', '.join(plan[s]) or '-'}")
    cls_desc = ", ".join(
        f"{(CLASSES[i] if i < len(CLASSES) else 'cls' + str(i))}={v}"
        for i, v in sorted(cls_count.items())) if cls_count else "（无框）"
    print("\n四类框数：" + cls_desc)

    seen = defaultdict(set)
    for s in SPLITS:
        for n in plan[s]:
            seen[n].add(s)
    leaked = [n for n, ss in seen.items() if len(ss) > 1]
    print(f"片段泄漏检查：{'通过，无视频组跨 split' if not leaked else '失败 ' + str(leaked)}")
    for s in ("val", "test"):
        if plan[s] and sum(pos_count.get(n, 0) for n in plan[s]) == 0:
            print(f"[严重] {s} 集没有任何含标志图片 → P/R/mAP50 无法计算，"
                  f"请换 --seed 或补标含标志帧")

    if args.dry_run:
        print("\n[dry-run] 未写入任何文件")
        return 0

    # 只写不删：mkdir + copy2，绝不 unlink / move / 清空目标目录
    for sub in ("images", "labels"):
        for s in SPLITS:
            (out_root / sub / s).mkdir(parents=True, exist_ok=True)
    existing = sum(
        len([f for f in (out_root / sub / s).iterdir() if f.is_file()])
        for sub in ("images", "labels") for s in SPLITS)
    if existing:
        print(f"[提示] 目标目录已有 {existing} 个文件；本脚本只覆盖同名文件，"
              f"不删除其它文件（换 seed 重跑前请自行核对）。")
    for s in SPLITS:
        for n in plan[s]:
            for img, lbl in groups[n]:
                shutil.copy2(img, out_root / "images" / s / img.name)
                shutil.copy2(lbl, out_root / "labels" / s / lbl.name)
    print(f"\n已写入 {out_root}（复制，只写不删）")
    print("下一步：python tools/train_yolo.py --data config/marker.yaml --name markers")
    return 0


if __name__ == "__main__":
    sys.exit(main())
