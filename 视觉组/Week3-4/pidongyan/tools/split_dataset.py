"""按「独立视频片段」划分 train / val / test，并复制到 YOLO 目录结构。

为什么不能用随机划分：
  同一段视频的相邻帧高度相似，若被打散到三个集合，测试集等于变相泄漏，
  指标会虚高。考核明确要求按独立视频片段（独立采集轮次）划分。

分组依据：抽帧文件名 <视频名>_f<帧号>.jpg 中的 <视频名> 前缀。
  因此请务必用 tools/extract_frames.py 抽帧，不要自行改名。

用法：
  python tools/split_dataset.py --dry-run          # 先看划分结果，不动文件
  python tools/split_dataset.py                    # 正式划分（复制）
  python tools/split_dataset.py --ratio 0.7 0.2 0.1 --seed 0
  python tools/split_dataset.py --labels frames_pool --move     # 移动而非复制（省空间）

前提：图片和对应 .txt 标签同名（LabelImg / X-AnyLabeling 默认行为）。
"""
import argparse
import random
import shutil
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
SPLITS = ("train", "val", "test")


def group_key(stem: str) -> str:
    """ball_run01_f000123 -> ball_run01；无 _f<数字> 后缀时整名作为组。"""
    head, sep, tail = stem.rpartition("_f")
    if sep and tail.isdigit():
        return head
    return stem


def collect(images_dir: Path, labels_dir: Path):
    groups = defaultdict(list)
    pos_count = defaultdict(int)          # 每组「含正样本(非空 txt)」的图片数
    missing = []
    for img in sorted(images_dir.iterdir()):
        if img.suffix.lower() not in IMG_EXTS:
            continue
        lbl = labels_dir / (img.stem + ".txt")
        if not lbl.exists():
            missing.append(img.name)
            continue
        g = group_key(img.stem)
        groups[g].append((img, lbl))
        try:
            if lbl.read_text(encoding="utf-8").strip():
                pos_count[g] += 1
        except OSError:
            pass
    return groups, missing, pos_count


def allocate(groups, ratio, seed, min_per_split=1, pos_count=None):
    """按图片数量比例分配视频组，保证每组只属于一个 split。

    pos_count 非空时启用「正样本保障」：
      * 纯空场组（0 个含球框）强制全部进 train —— 负样本的作用是训练时压误检，
        放进 val/test 只会稀释分母，且若 test 全是无球图，mAP50 / P / R 无法计算。
      * 只对含球组做 70/20/10 比例划分，并保证 val / test 各至少有 1 个含球组。
    """
    names = list(groups.keys())
    random.Random(seed).shuffle(names)

    sizes = {n: len(groups[n]) for n in names}
    total = sum(sizes.values())
    if total == 0:
        raise SystemExit("没有可用样本（检查图片与标签是否同名且在同一目录）")

    if pos_count is None:
        pool, forced_train = names, []
    else:
        pool = [n for n in names if pos_count.get(n, 0) > 0]
        forced_train = [n for n in names if pos_count.get(n, 0) == 0]
        if not pool:
            raise SystemExit("所有视频组都没有含球框，无法划分出有效的验证/测试集")

    plan = {s: [] for s in SPLITS}
    plan["train"].extend(forced_train)
    if forced_train:
        print(f"[正样本保障] 纯空场组 {len(forced_train)} 个强制进 train："
              f"{', '.join(forced_train)}")

    # 只对含球组按比例确定归属：对累计图片数比例落在哪个区间
    base = sum(sizes[n] for n in pool) or 1
    bounds = [ratio[0], ratio[0] + ratio[1]]
    cumulative = 0
    for n in pool:
        pos = (cumulative + sizes[n] / 2) / base  # 用组中心判定归属
        if pos < bounds[0]:
            plan["train"].append(n)
        elif pos < bounds[1]:
            plan["val"].append(n)
        else:
            plan["test"].append(n)
        cumulative += sizes[n]

    # 保底：val/test 至少各有 1 个含球组；train 至少 1 组
    for s in SPLITS:
        if pos_count is not None and s in ("val", "test"):
            ok = any(pos_count.get(n, 0) > 0 for n in plan[s])
        else:
            ok = len(plan[s]) >= min_per_split
        if ok:
            continue
        donors = sorted((k for k in SPLITS if k != s),
                        key=lambda k: -len(plan[k]))
        for donor in donors:
            # 只搬「含球组」，且 donor 搬完还得剩至少 1 组
            movable = [n for n in plan[donor]
                       if pos_count is None or pos_count.get(n, 0) > 0]
            if movable and len(plan[donor]) > 1:
                pick = movable[0]
                plan[donor].remove(pick)
                plan[s].append(pick)
                print(f"[保底] 从 {donor} 挪含球组 {pick} 到 {s}")
                break
    return plan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default=str(ROOT / "frames_pool"), help="已标注图片目录")
    ap.add_argument("--labels", default=None, help="标签目录，默认与图片同目录")
    ap.add_argument("--out", default=str(ROOT / "dataset"), help="YOLO 数据集根目录")
    ap.add_argument("--ratio", nargs=3, type=float, default=[0.7, 0.2, 0.1],
                    metavar=("TRAIN", "VAL", "TEST"))
    ap.add_argument("--seed", type=int, default=0, help="随机种子，保证可复现")
    ap.add_argument("--move", action="store_true", help="移动而非复制")
    ap.add_argument("--dry-run", action="store_true", help="只打印划分结果")
    args = ap.parse_args()

    images_dir = Path(args.images)
    labels_dir = Path(args.labels) if args.labels else images_dir
    out_root = Path(args.out)

    if abs(sum(args.ratio) - 1.0) > 1e-6:
        sys.exit(f"比例之和必须为 1，当前 {sum(args.ratio)}")

    groups, missing, pos_count = collect(images_dir, labels_dir)
    if not groups:
        sys.exit(f"{images_dir} 下没有找到「图片+同名txt」的样本")

    print(f"样本 {sum(len(v) for v in groups.values())} 张，来自 {len(groups)} 个视频组")
    if missing:
        print(f"[警告] {len(missing)} 张图片没有对应标签，已跳过（未标注的球会被当成背景）：")
        for name in missing[:5]:
            print(f"    {name}")
        if len(missing) > 5:
            print(f"    ... 共 {len(missing)} 张")

    plan = allocate(groups, args.ratio, args.seed, pos_count=pos_count)

    print("\n划分结果：")
    print(f"{'split':<8}{'视频组':<8}{'图片数':<8}{'含球':<8}{'占比':<8}视频组名")
    grand = sum(len(v) for v in groups.values())
    for s in SPLITS:
        cnt = sum(len(groups[n]) for n in plan[s])
        npos = sum(pos_count.get(n, 0) for n in plan[s])
        pct = cnt / grand * 100 if grand else 0
        print(f"{s:<8}{len(plan[s]):<8}{cnt:<8}{npos:<8}{pct:>5.1f}%  {', '.join(plan[s])}")

    # 泄漏自检
    seen = defaultdict(set)
    for s in SPLITS:
        for n in plan[s]:
            seen[n].add(s)
    leaked = [n for n, ss in seen.items() if len(ss) > 1]
    print(f"\n片段泄漏检查：{'通过，无视频组跨 split' if not leaked else '失败 ' + str(leaked)}")

    empty = [s for s in SPLITS if not plan[s]]
    if empty:
        print(f"[警告] {', '.join(empty)} 为空。视频组太少（当前 {len(groups)} 个），"
              f"考核要求 ≥5 段独立视频，请补拍后再划分。")

    for s in ("val", "test"):
        if plan[s] and sum(pos_count.get(n, 0) for n in plan[s]) == 0:
            print(f"[严重] {s} 集没有任何含球图片 → P/R/mAP50 无法计算！"
                  f"请换 --seed 或补标含球帧。")

    if args.dry_run:
        print("\n[dry-run] 未写入任何文件")
        return

    # 重跑前清空三个 split 目录里的旧产物：否则换 --seed / 补标后重跑时，
    # 上一轮的文件会残留，两个划分方案混在一起（图片与标签还会错配）。
    for s in SPLITS:
        for sub, exts in (("images", IMG_EXTS), ("labels", {".txt"})):
            d = out_root / sub / s
            if d.exists():
                for f in d.iterdir():
                    if f.is_file() and f.suffix.lower() in exts:
                        f.unlink()
        (out_root / "images" / s).mkdir(parents=True, exist_ok=True)
        (out_root / "labels" / s).mkdir(parents=True, exist_ok=True)

    for s in SPLITS:
        for n in plan[s]:
            for img, lbl in groups[n]:
                dst_img = out_root / "images" / s / img.name
                dst_lbl = out_root / "labels" / s / lbl.name
                if args.move:
                    shutil.move(str(img), str(dst_img))
                    shutil.move(str(lbl), str(dst_lbl))
                else:
                    shutil.copy2(str(img), str(dst_img))
                    shutil.copy2(str(lbl), str(dst_lbl))

    print(f"\n已写入 {out_root}（{'移动' if args.move else '复制'}）")
    print("下一步：python tools/check_labels.py --dataset dataset   # 抽查画框效果")


if __name__ == "__main__":
    main()
