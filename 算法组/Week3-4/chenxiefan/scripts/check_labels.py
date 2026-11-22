#!/usr/bin/env python3

import argparse
import glob
import os

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="数据集根目录，例如 dataset/ball")
    ap.add_argument("--classes", type=int, default=0, help="类别总数，0 表示自动取 data.yaml")
    args = ap.parse_args()

    root = args.data
    n_classes = args.classes
    yaml_path = os.path.join(root, "data.yaml")
    if n_classes <= 0 and os.path.exists(yaml_path):
        import yaml
        with open(yaml_path, encoding="utf-8") as f:
            n_classes = len((yaml.safe_load(f) or {}).get("names", {}))
    if n_classes <= 0:
        n_classes = 1

    total_img = 0
    total_lbl = 0
    empty_lbl = 0
    bad = []
    for split in ("train", "val"):
        img_dir = os.path.join(root, "images", split)
        lbl_dir = os.path.join(root, "labels", split)
        if not os.path.isdir(img_dir):
            continue
        for img in sorted(glob.glob(os.path.join(img_dir, "*"))):
            base = os.path.splitext(os.path.basename(img))[0]
            lbl = os.path.join(lbl_dir, base + ".txt")
            total_img += 1
            if not os.path.exists(lbl):
                bad.append(f"缺标签：{img}")
                continue
            total_lbl += 1
            lines = [ln.strip() for ln in open(lbl, encoding="utf-8") if ln.strip()]
            if not lines:
                empty_lbl += 1
                continue
            for ln in lines:
                parts = ln.split()
                if len(parts) != 5:
                    bad.append(f"格式错：{lbl} -> {ln}")
                    continue
                try:
                    cid = int(parts[0])
                    vals = [float(v) for v in parts[1:]]
                except ValueError:
                    bad.append(f"数值错：{lbl} -> {ln}")
                    continue
                if not (0 <= cid < n_classes):
                    bad.append(f"类别号越界：{lbl} -> {ln}")
                if any(v < 0 or v > 1 for v in vals):
                    bad.append(f"坐标越界：{lbl} -> {ln}")
                if vals[2] <= 0 or vals[3] <= 0:
                    bad.append(f"宽高为 0：{lbl} -> {ln}")

    print(f"图片 {total_img} 张，标签 {total_lbl} 个，空标签 {empty_lbl} 个")
    if bad:
        print(f"发现 {len(bad)} 个问题（前 20 条）：")
        for b in bad[:20]:
            print(" -", b)
    else:
        print("检查通过")
    return 1 if bad else 0

if __name__ == "__main__":
    raise SystemExit(main())
