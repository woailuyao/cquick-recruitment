"""统计 ball 数据集真实数量：图片/标签配对、空帧、实例数、标签越界校验。"""
import os

ROOT = r"D:\W34_dataset\ball"
IMG = os.path.join(ROOT, "images")
LBL = os.path.join(ROOT, "labels")

grand = {"img": 0, "lbl": 0, "empty": 0, "inst": 0, "bad": 0}

for split in ["train", "val", "test"]:
    img_dir = os.path.join(IMG, split)
    lbl_dir = os.path.join(LBL, split)
    imgs = [f for f in os.listdir(img_dir) if f.lower().endswith(".jpg")]
    lbls = [f for f in os.listdir(lbl_dir) if f.lower().endswith(".txt") and f != "classes.txt"]
    n_empty = 0
    n_inst = 0
    bad = []
    for f in lbls:
        p = os.path.join(lbl_dir, f)
        with open(p, "r", encoding="utf-8") as fh:
            lines = [ln.strip() for ln in fh if ln.strip()]
        if not lines:
            n_empty += 1
            continue
        for ln in lines:
            n_inst += 1
            parts = ln.split()
            if len(parts) != 5:
                bad.append((f, "列数!=5"))
                continue
            cx, cy, w, h = map(float, parts[1:])
            if not (0 <= cx <= 1 and 0 <= cy <= 1 and 0 < w <= 1 and 0 < h <= 1):
                bad.append((f, f"越界 {cx},{cy},{w},{h}"))
    # 配对检查：图片名去后缀 vs 标签名去后缀
    img_stems = {os.path.splitext(f)[0] for f in imgs}
    lbl_stems = {os.path.splitext(f)[0] for f in lbls}
    missing_lbl = sorted(img_stems - lbl_stems)
    missing_img = sorted(lbl_stems - img_stems)
    print(f"[{split}] 图片={len(imgs)} 标签={len(lbls)} 空帧={n_empty} 实例={n_inst} 异常={len(bad)}")
    print(f"        有图无标签={len(missing_lbl)} 有标签无图={len(missing_img)}")
    if bad[:3]:
        print(f"        异常样例: {bad[:3]}")
    grand["img"] += len(imgs); grand["lbl"] += len(lbls)
    grand["empty"] += n_empty; grand["inst"] += n_inst; grand["bad"] += len(bad)

print("==== 合计 ====")
print(grand)
