"""Dataset audit + regroup train/val/test by independent video segment.

Why: split_markers.py splits by random image, so adjacent frames of the same
video land in both train and val (that is why mAP50 was 0.995), and there is no
test split at all.

Usage:
    python scripts/audit_dataset.py            # stats only, changes nothing
    python scripts/audit_dataset.py --apply    # rewrite images/{train,val,test}
"""

from __future__ import annotations

import argparse
import random
import re
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))
DEFAULT_DS = ROOT / "dataset"
IMG_SUFFIX = {".jpg", ".jpeg", ".png", ".bmp"}


def group_of(stem: str) -> str:
    """L_000015 -> L ; live_L_0022 -> live_L ; T1_000048 -> T1 ; cam1_0007 -> cam1"""
    m = re.match(r"^(.+?)[-_]\d+$", stem)
    return m.group(1) if m else stem


def read_label(path):
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) >= 5:
            rows.append((int(parts[0]), *map(float, parts[1:5])))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", default=str(DEFAULT_DS))
    ap.add_argument("--images", default=None, help="default dataset/images/selected")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--val-ratio", type=float, default=0.20)
    ap.add_argument("--test-ratio", type=float, default=0.10)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--assign", default=None,
                    help="手动指定分组归属，例如 "
                         "\"train=L,T,X;val=live_L,T1;test=X1\"（优先于自动划分）")
    args = ap.parse_args()

    ds = Path(args.dataset)
    img_dir = Path(args.images) if args.images else ds / "images" / "selected"
    if not img_dir.exists():
        raise SystemExit(f"image dir not found: {img_dir}")

    images = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMG_SUFFIX)
    if not images:
        raise SystemExit("no images")

    classes = ["LCross", "TCross", "XCross"]
    cpath = ds / "classes.txt"
    if cpath.exists():
        # classes.txt 带 UTF-8 BOM，用 utf-8-sig 去掉，否则类别名会变成 "\ufeffLCross"
        classes = [c.strip() for c in
                   cpath.read_text(encoding="utf-8-sig").splitlines() if c.strip()]

    per_class = Counter()
    per_group = defaultdict(Counter)
    areas, n_inst, missing = [], 0, []
    for img in images:
        lbl = img.with_suffix(".txt")
        if not lbl.exists():
            missing.append(img.name)
            continue
        g = group_of(img.stem)
        for cls, xc, yc, w, h in read_label(lbl):
            per_class[cls] += 1
            per_group[g][cls] += 1
            areas.append(w * h)
            n_inst += 1

    print("=" * 62)
    print(f"image dir : {img_dir}")
    print(f"images    : {len(images)}  labelled: {len(images)-len(missing)}  "
          f"missing label: {len(missing)}")
    print(f"instances : {n_inst}")
    print(f"classes   : {classes}")
    print("-" * 62)
    for i, name in enumerate(classes):
        print(f"  {i} {name:<13} {per_class.get(i,0):4d} instances")
    if areas:
        a = sorted(areas)
        print(f"box area ratio: median {a[len(a)//2]:.3f}  min {a[0]:.3f}  max {a[-1]:.3f}")
        big = sum(x > 0.15 for x in areas)
        print(f"  large boxes (>15% of image): {big}/{len(areas)}"
              "  - many large boxes means close-up only, far/small will be missed")
    print("-" * 62)
    print(f"video segments (groups): {len(per_group)}")
    for g, c in sorted(per_group.items()):
        tot = sum(c.values())
        print(f"  {g:<10} {tot:4d} instances   " +
              "  ".join(f"{classes[k][:1]}={v}" for k, v in sorted(c.items())))

    print("-" * 62)
    warn = []
    if len(images) < 300:
        warn.append(f"images {len(images)} < 300")
    if len(per_group) < 5:
        warn.append(f"independent videos {len(per_group)} < 5")
    if missing:
        warn.append(f"{len(missing)} images without labels: {missing[:3]}")
    for w in warn:
        print(f"! {w}")
    if not warn:
        print("dataset size check passed")

    if not args.apply:
        print("\n(add --apply to actually regroup the files)")
        return

    groups = sorted(per_group.keys())
    if len(groups) < 3:
        print(f"\n! only {len(groups)} video segments, cannot build train/val/test")
        return

    rng = random.Random(args.seed)
    sizes = {g: sum(per_group[g].values()) for g in groups}
    order = groups[:]
    rng.shuffle(order)
    order.sort(key=lambda g: -sizes[g])

    total = sum(sizes.values())
    targets = {"test": args.test_ratio, "val": args.val_ratio}
    targets["train"] = max(0.0, 1 - args.val_ratio - args.test_ratio)
    got = {"train": 0, "val": 0, "test": 0}
    assign = {}
    if args.assign:
        for part in args.assign.split(";"):
            if not part.strip():
                continue
            split, names = part.split("=")
            split = split.strip()
            if split not in got:
                raise SystemExit(f"unknown split {split!r}")
            for g in [x.strip() for x in names.split(",") if x.strip()]:
                if g not in groups:
                    raise SystemExit(f"group {g!r} not found; available: {groups}")
                if g in assign:
                    raise SystemExit(f"group {g!r} assigned twice")
                assign[g] = split
                got[split] += sizes[g]
        missing = [g for g in groups if g not in assign]
        for g in missing:
            print(f"  [warn] group {g} not assigned, putting into train")
            assign[g] = "train"
            got["train"] += sizes[g]
    else:
        for g in order:
            best_split, best_gap = "train", -1e9
            for s, tgt in targets.items():
                gap = tgt * total - got[s]
                if s != "train" and got[s] == 0 and len(order) >= 3:
                    gap += total * 0.5
                if gap > best_gap:
                    best_gap, best_split = gap, s
            assign[g] = best_split
            got[best_split] += sizes[g]

    print("\nsplit:")
    for g in groups:
        # 每组分到的类别统计，方便确认没把某个类别漏掉
        cls = per_group[g]
        summary = " ".join(f"{classes[k][:1]}={v}" for k, v in sorted(cls.items()))
        print(f"  {g:<10} -> {assign[g]:<5} ({sum(cls.values()):3d} 实例  {summary})")
    for s in ("train", "val", "test"):
        print(f"  {s:<6} {got[s]:4d} instances ({got[s]/max(1,total)*100:.0f}%)")
        if got[s] == 0:
            print(f"    ! {s} 是空的，训练/评估会报错")
    if got["val"] / max(1, total) > 0.30 or got["test"] / max(1, total) > 0.20:
        print("! too few video segments to hit 70/20/10 precisely;"
              " 8-10 segments is recommended")

    for s in ("train", "val", "test"):
        for sub in ("images", "labels"):
            d = ds / sub / s
            if d.exists():
                shutil.rmtree(d)
            d.mkdir(parents=True, exist_ok=True)

    counts = Counter()
    for img in images:
        lbl = img.with_suffix(".txt")
        if not lbl.exists():
            continue
        s = assign[group_of(img.stem)]
        shutil.copy2(img, ds / "images" / s / img.name)
        shutil.copy2(lbl, ds / "labels" / s / lbl.name)
        counts[s] += 1

    data = {
        "path": str(ds).replace("\\", "/"),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "nc": len(classes),
        "names": {i: n for i, n in enumerate(classes)},
    }
    (ds / "data.yaml").write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(f"\nwritten {ds/'data.yaml'}")
    print(f"train={counts['train']}  val={counts['val']}  test={counts['test']}")
    print("note: metrics will drop compared to the random split - that is the truth")


if __name__ == "__main__":
    main()