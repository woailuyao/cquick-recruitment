"""生成「伪远距小球」训练样本，缓解尺度外 OOD（不动正式 dataset/）。

背景：train 的球框中位 169.6 px，test 只有 82.5 px，且训练集中没有任何
远距小球片段。调 imgsz / 优化器 / mosaic 都无法凭空制造小目标样本
（imgsz 会把 train 和 test 一起缩放，相对尺度差不变）。

做法（只用 train 自己的图片，绝不碰 val/test，绝不碰 test 片段）：
  1. 把原 dataset/ 完整复制到 dataset_aug/（正式划分保持原样、可复现 v1）；
  2. 对 train 里每张含球图：整图按 s∈[0.40,0.65] 缩小 → 球变小；
  3. 背景用**同一张图的强模糊版**（色调/纹理一致，原球被糊掉），
     把缩小图随机放进去。这样没有颜色突变，也不会像贴图那样把球羽化掉；
  4. 同步换算 YOLO 标签坐标，写入 dataset_aug/images|labels/train/；
  5. 生成 config/data_aug.yaml，用 v1 的**基线超参**训练 v3，
     这样 v1/v3 唯一变量就是「有没有小尺度训练样本」，结论干净。

用法：
  python tools/make_small_ball_aug.py                 # 生成 dataset_aug + data_aug.yaml
  python tools/make_small_ball_aug.py --dry-run       # 只打印计划

本脚本**只写不删**：目标目录不存在就复制创建；已存在则覆盖同名文件，
不删除任何已有目录或文件（2026-11-20 按用户要求移除所有删除逻辑）。
第一版有拼接痕迹的旧产物已整体移至 `dataset_aug_rejected/` 保留，未删除。

注意：生成的图片文件名带 _sb0 后缀，作为独立「视频组」只存在于 train。
**不要再对 dataset_aug 跑 split_dataset.py**（增强图与原图同源，重新划分会造成泄漏）；
val/test 是原样复制，指标可与 v1 直接对比。
"""
import argparse
import random
import shutil
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
SUFFIX = "_sb0"
MIN_BALL_PX = 20.0   # 缩小后小于 20px 的球直接丢弃，避免教坏模型


def read_boxes(lbl_path):
    txt = lbl_path.read_text(encoding="utf-8").strip() if lbl_path.exists() else ""
    out = []
    for line in txt.splitlines():
        p = line.split()
        if len(p) >= 5:
            out.append(tuple(float(x) for x in p[:5]))
    return out


def make_one(img, boxes, rng, s):
    """整图缩小 + 同图模糊背景。返回 (增强图, 新标签行列表)。"""
    H, W = img.shape[:2]
    sw = max(8, int(round(W * s)))
    sh = max(8, int(round(H * s)))
    small = cv2.resize(img, (sw, sh), interpolation=cv2.INTER_AREA)

    # 背景：同一张图强模糊，颜色/亮度与前景一致，且原球被糊成色块
    bg = cv2.GaussianBlur(img, (0, 0), sigmaX=30, sigmaY=30)
    x0 = rng.randint(0, W - sw)
    y0 = rng.randint(0, H - sh)

    rows = []
    for (cls, cx, cy, w, h) in boxes:
        new_bw, new_bh = w * W * s, h * H * s
        if new_bw < MIN_BALL_PX or new_bh < MIN_BALL_PX:
            return None                      # 太小的目标不生成
        # 整图等比缩小 → 归一化中心不变，只是加放置偏移
        n_cx = (cx * sw + x0) / W
        n_cy = (cy * sh + y0) / H
        n_w, n_h = w * s, h * s
        if not (n_w / 2 < n_cx < 1 - n_w / 2 and n_h / 2 < n_cy < 1 - n_h / 2):
            return None                      # 球被推出画面
        rows.append(f"{int(cls)} {n_cx:.6f} {n_cy:.6f} {n_w:.6f} {n_h:.6f}")

    # 硬贴即可：前景/背景只有「清晰度」差异，没有颜色突变；
    # 不羽化是为了保证球不被边缘渐变淡化（标签与图像必须严格对应）。
    bg[y0:y0 + sh, x0:x0 + sw] = small
    return bg, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(ROOT / "dataset"))
    ap.add_argument("--dst", default=str(ROOT / "dataset_aug"))
    ap.add_argument("--config", default=str(ROOT / "config" / "data_aug.yaml"))
    ap.add_argument("--factor-min", type=float, default=0.40)
    ap.add_argument("--factor-max", type=float, default=0.65)
    ap.add_argument("--per-image", type=int, default=1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    src, dst = Path(args.src), Path(args.dst)
    rng = random.Random(args.seed)
    pos_imgs = []
    for p in sorted((src / "images" / "train").iterdir()):
        if p.suffix.lower() not in IMG_EXTS:
            continue
        lbl = src / "labels" / "train" / (p.stem + ".txt")
        boxes = read_boxes(lbl)
        if boxes:
            pos_imgs.append((p, boxes))

    print(f"train 含球图 {len(pos_imgs)} 张")
    print(f"缩放因子 s ∈ [{args.factor_min}, {args.factor_max}]，每图 {args.per_image} 张增强")
    if args.dry_run:
        print("[dry-run] 预计新增", len(pos_imgs) * args.per_image, "张，未写任何文件")
        return

    # 完整复制正式划分，val/test 一个字节都不改。
    # dirs_exist_ok=True：目录已存在时只覆盖同名文件，不删除任何东西。
    shutil.copytree(src, dst, ignore=shutil.ignore_patterns("*.cache"),
                    dirs_exist_ok=True)

    made, skipped, widths = 0, 0, []
    for img_path, boxes in pos_imgs:
        img = cv2.imread(str(img_path))
        if img is None:
            skipped += 1
            continue
        H, W = img.shape[:2]
        for k in range(args.per_image):
            s = rng.uniform(args.factor_min, args.factor_max)
            res = make_one(img, boxes, rng, s)
            if res is None:
                skipped += 1
                continue
            aug, rows = res
            stem = (f"{img_path.stem}{SUFFIX}" if k == 0
                    else f"{img_path.stem}{SUFFIX[:-1]}{k}")
            cv2.imwrite(str(dst / "images" / "train" / (stem + ".jpg")), aug)
            (dst / "labels" / "train" / (stem + ".txt")).write_text(
                "\n".join(rows) + "\n", encoding="utf-8")
            for r in rows:
                widths.append(float(r.split()[3]) * W)
            made += 1

    cfg = Path(args.config)
    cfg.parent.mkdir(parents=True, exist_ok=True)
    cfg.write_text(
        "# 任务一 · 尺度增强数据集（v3 用）\n"
        "# 由 tools/make_small_ball_aug.py 生成：dataset/ 的完整副本 + train 里\n"
        "# 新增伪远距小球样本。val/test 与原划分完全一致，保证指标可比。\n"
        "# 注意：不要再对本目录跑 split_dataset.py（增强图与原图同源）。\n"
        f"path: {dst.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n\n"
        "nc: 1\n"
        "names:\n  0: ball\n",
        encoding="utf-8")

    if widths:
        widths.sort()
        print(f"\n已生成 {made} 张增强图（跳过 {skipped}），写入 {dst}")
        print(f"增强球框宽度 px：min {min(widths):.1f} / 中位 "
              f"{widths[len(widths)//2]:.1f} / max {max(widths):.1f}"
              f"   （目标：覆盖 test 的 82.5 px）")
    print(f"配置已写入 {cfg}")
    print(f"\n下一步（基线超参，唯一变量是数据）：")
    print(f"  python tools/train_yolo.py --data {cfg.relative_to(ROOT).as_posix()} --name train_v3")


if __name__ == "__main__":
    main()
