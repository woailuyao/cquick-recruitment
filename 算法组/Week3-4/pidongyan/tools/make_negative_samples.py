"""生成「类球干扰」空标签负样本，构建 dataset_aug2（任务一 v4 实验用）。

背景
----
v3 在独立 test 集误检从 v1 的 4 涨到 35，主因是背景里的蓝色地球仪圆形图案、
栏杆圆环、设备箱扣、白色圆形台面被当成球；而 v1/v3 的 68 张负样本**全是空场地**，
没覆盖类球干扰物。这些干扰物在 marker_run / pose / field_check 视频段里本来就有，
**不需要补拍、不需要相机**。

依据（刻意不用模型筛球）
------------------------
frames_pool_markers/ 的 175 张帧全部来自 11 个无球视频段
（marker_run01~05、pose_A/A2/B/C、field_check*），与任务一 ball_run* 段严格分离
（ball 帧在 frames_pool/）。按视频组等间隔抽帧，全部写成空标签。
**不用 v3 检测来筛「有没有球」** —— v3 在这些帧上的高置信误检恰恰是要保留的
hard negative，用模型筛会把它们删掉，等于把药倒掉。
2026-11-20 已目视抽查 marker_run04b / pose_C / field_check 三组代表帧，均确认无球。

红线
----
- 只写不删；不碰 dataset/、dataset_aug/（v3 必须可复现），不碰 val/test。
- 输出独立目录 dataset_aug2/ + config/data_aug2.yaml，训练用 --name train_v4，
  **不覆盖 v3 权重**。
- 帧未经逐张人工复核，清单写入 logs/negative_samples_20261120.txt，建议醒来抽检 5 张。
- 幂等：同名 _neg 文件已存在则跳过，重复运行不会累积。

用法
----
  python tools/make_negative_samples.py
  python tools/make_negative_samples.py --per-group 4
  python tools/make_negative_samples.py --dry-run
"""
import argparse
import re
import shutil
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def group_key(stem):
    return re.sub(r"_f\d+$", "", stem)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default="frames_pool_markers")
    ap.add_argument("--base", default="dataset_aug", help="v3 数据集，原样复制")
    ap.add_argument("--out", default="dataset_aug2")
    ap.add_argument("--per-group", type=int, default=4, help="每个视频组最多抽几张")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    src = ROOT / args.src
    base = ROOT / args.base
    out = ROOT / args.out
    imgs = sorted(src.glob("*.jpg"))
    if not imgs:
        raise SystemExit(f"{src} 里没有 jpg")

    groups = defaultdict(list)
    for p in imgs:
        groups[group_key(p.stem)].append(p)

    picked = []
    for g in sorted(groups):
        fs = groups[g]
        k = min(args.per_group, len(fs))
        if k <= 0:
            continue
        # 等间隔取，避开只取开头
        idx = [round(i * (len(fs) - 1) / max(k - 1, 1)) for i in range(k)] if k > 1 else [0]
        for i in sorted(set(idx)):
            picked.append(fs[i])

    print(f"源目录 {src}：{len(imgs)} 张 / {len(groups)} 个无球视频组")
    print(f"每组最多 {args.per_group} 张 -> 抽出 {len(picked)} 张负样本")
    for g in sorted(groups):
        sel = [p.name for p in picked if group_key(p.stem) == g]
        print(f"  {g:38s} {len(sel):2d} 张")
    if args.dry_run:
        print("\n[dry-run] 未写文件")
        return 0

    # 1) 复制 v3 数据集（只写不删）
    if not out.exists():
        shutil.copytree(base, out)
        print(f"\n已复制 {base} -> {out}")
    else:
        print(f"\n[提示] {out} 已存在，沿用其内容，仅追加 _neg 负样本")

    (out / "images" / "train").mkdir(parents=True, exist_ok=True)
    (out / "labels" / "train").mkdir(parents=True, exist_ok=True)

    added, skipped = [], []
    for p in picked:
        stem = p.stem + "_neg"
        dst_img = out / "images" / "train" / (stem + ".jpg")
        dst_lbl = out / "labels" / "train" / (stem + ".txt")
        if dst_img.exists() and dst_lbl.exists():
            skipped.append(stem)
            continue
        shutil.copy2(p, dst_img)
        dst_lbl.write_text("", encoding="utf-8")   # 空标签 = 负样本
        added.append(stem)

    # 2) 数据集配置
    cfg = ROOT / "config" / "data_aug2.yaml"
    cfg.write_text(
        "# 任务一 · 尺度增强 + 类球干扰负样本（v4 实验用）\n"
        "# 由 tools/make_negative_samples.py 生成：dataset_aug/ 完整副本 + train 追加\n"
        f"# {len(added) + len(skipped)} 张无球帧（空标签）。val/test 与 dataset/ 完全一致。\n"
        "# 不要再对本目录跑 split_dataset.py。\n"
        f"path: {out.as_posix()}\n"
        "train: images/train\nval: images/val\ntest: images/test\n\n"
        "nc: 1\nnames:\n  0: ball\n", encoding="utf-8")

    # 3) 清单
    log = ROOT / "logs" / "negative_samples_20261120.txt"
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(
        "# 任务一类球干扰负样本清单（v4 实验）\n"
        f"# 源：{src}（11 个无球视频段，与 ball_run* 严格分离）\n"
        "# 全部为空标签；未经逐张人工复核，建议抽检 5 张确认无球。\n"
        + "\n".join(f"{s}  <- {src.name}/{s.replace('_neg', '')}.jpg" for s in added + skipped)
        + f"\n# 新增 {len(added)} 张，跳过（已存在）{len(skipped)} 张\n",
        encoding="utf-8")

    n_train = len(list((out / "images" / "train").glob("*.jpg")))
    print(f"新增 {len(added)} 张，跳过 {len(skipped)} 张；train 共 {n_train} 张")
    print(f"配置 -> {cfg}")
    print(f"清单 -> {log}")
    print("\n下一步（不覆盖 v3）：")
    print("  python tools/train_yolo.py --data config/data_aug2.yaml --name train_v4")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
