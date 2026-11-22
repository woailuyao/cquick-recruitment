# -*- coding: utf-8 -*-
"""prepare_landmark_dataset.py —— 把 labelImg 的工作目录拆成 YOLO 训练集

labelImg 要求「图片与 txt 同目录」，而 ultralytics 要求「images/ 与 labels/
分目录、且各含 train/val 子目录」。本脚本负责这个转换。

关键设计：**按视频整段划分 train/val**，不按帧随机抽。
同一段视频的相邻帧几乎一模一样，随机抽会让验证集里塞满训练集的"近重复
图"，mAP 虚高、看不出真实泛化能力。

用法：
  python prepare_landmark_dataset.py
  python prepare_landmark_dataset.py --val-videos MC1 M2 M3 M11
  python prepare_landmark_dataset.py --src D:\\captures\\_anno\\labelimg_work ^
                                          D:\\captures\\_anno\\fixed_work ^
                                     --dst D:\\captures\\landmarks_v2
"""
import argparse
import os
import shutil
from collections import Counter, defaultdict

IMG_EXT = (".jpg", ".jpeg", ".png")
CLASS_NAMES = ["L", "T"]


def video_of(stem):
    """M1_00001 -> M1；MC2_00123 -> MC2。"""
    return stem.rsplit("_", 1)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", nargs="+",
                    default=[r"D:\captures\_anno\labelimg_work",
                             r"D:\captures\_anno\fixed_work"],
                    help="一个或多个 labelImg 工作目录（图片与 txt 同目录）")
    ap.add_argument("--dst", default=r"D:\captures\landmarks_v2")
    ap.add_argument("--val-videos", nargs="*",
                    default=["MC1", "M2", "M3", "M11"],
                    help="整段留作验证集的视频前缀；其余进训练集")
    ap.add_argument("--clean", action="store_true", help="先清空目标目录")
    ap.add_argument("--skip-empty", action="store_true",
                    help="丢弃没有框的帧（默认保留，作为背景负样本）")
    ap.add_argument("--yaml-path", default=None,
                    help="data.yaml 里 path 字段的写法。默认自动转换："
                         "D:////xxx -> /mnt/d/xxx（训练在 WSL 里跑）")
    args = ap.parse_args()

    for src in args.src:
        if not os.path.isdir(src):
            print("源目录不存在:", src)
            return 1
    if args.clean and os.path.isdir(args.dst):
        shutil.rmtree(args.dst)

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(args.dst, sub), exist_ok=True)

    # 汇总所有源目录的图片；任何一张图缺 txt 都直接报错，避免把
    # "用户还没标完" 的帧当成背景负样本混进训练集。
    all_imgs = []
    for src in args.src:
        imgs = sorted(f for f in os.listdir(src)
                      if f.lower().endswith(IMG_EXT))
        if not imgs:
            print("源目录没有图片:", src)
            return 1
        missing = [f for f in imgs
                   if not os.path.exists(os.path.join(
                       src, os.path.splitext(f)[0] + ".txt"))]
        if missing:
            print("以下 %d 张图还没有 YOLO 标签，先标注完再运行："
                  % len(missing))
            for f in missing[:20]:
                print("  ", os.path.join(src, f))
            if len(missing) > 20:
                print("   ...")
            return 1
        all_imgs.extend((src, f) for f in imgs)
    if not all_imgs:
        print("没有可用图片")
        return 1

    val_set = set(args.val_videos)
    stats = defaultdict(lambda: {"img": 0, "box": 0, "cls": Counter(),
                                 "empty": 0, "no_txt": 0})

    for src_dir, name in all_imgs:
        stem = os.path.splitext(name)[0]
        vid = video_of(stem)
        split = "val" if vid in val_set else "train"

        src_txt = os.path.join(src_dir, stem + ".txt")
        boxes = []
        if os.path.exists(src_txt):
            with open(src_txt, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    p = line.split()
                    if len(p) == 5:
                        try:
                            boxes.append((int(float(p[0])),
                                          [float(x) for x in p[1:]]))
                        except ValueError:
                            pass
        else:
            stats[split]["no_txt"] += 1

        if not boxes:
            stats[split]["empty"] += 1
            if args.skip_empty:
                continue

        # 图片
        shutil.copy2(os.path.join(src_dir, name),
                     os.path.join(args.dst, "images", split, name))
        # 标签（图片与标签必须同名，空标签也要写空文件，否则 yolov5/8
        # 会把这张图当成"无标签"直接跳过，等于白丢一个背景样本）
        dst_txt = os.path.join(args.dst, "labels", split, stem + ".txt")
        with open(dst_txt, "w", encoding="utf-8") as f:
            for cls, vals in boxes:
                f.write("%d %.6f %.6f %.6f %.6f\n" % (cls, *vals))

        stats[split]["img"] += 1
        stats[split]["box"] += len(boxes)
        for cls, _ in boxes:
            stats[split]["cls"][cls] += 1

    # data.yaml：ultralytics 认这个格式
    # 训练在 WSL 里跑，Windows 盘符路径（D://captures//...）Linux 读不到，
    # 必须转成 /mnt/d/... 形式；训练在 Windows 下跑时用 --yaml-path 覆盖。
    if args.yaml_path:
        root = args.yaml_path
    else:
        dst_norm = os.path.abspath(args.dst).replace("\\", "/")
        if len(dst_norm) > 1 and dst_norm[1] == ":":
            root = "/mnt/%s%s" % (dst_norm[0].lower(), dst_norm[2:])
        else:
            root = dst_norm
    yaml_path = os.path.join(args.dst, "data.yaml")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write("# 场地标志检测（L/T 两类），由 prepare_landmark_dataset.py 生成\n")
        f.write("path: %s\n" % root)
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("names:\n")
        for i, n in enumerate(CLASS_NAMES):
            f.write("  %d: %s\n" % (i, n))
        f.write("nc: %d\n" % len(CLASS_NAMES))

    # ---------- 汇总 ----------
    print("目标目录:", args.dst)
    print("验证集视频:", ", ".join(args.val_videos))
    print("")
    print("| 集合 | 图片 | 框 | L | T | 空标签帧 | 缺标签 |")
    print("|---|---|---|---|---|---|---|")
    for split in ("train", "val"):
        s = stats[split]
        print("| %s | %d | %d | %d | %d | %d | %d |"
              % (split, s["img"], s["box"], s["cls"][0], s["cls"][1],
                 s["empty"], s["no_txt"]))
    print("")
    print("data.yaml:", yaml_path)

    # 提醒：验证集缺某一类时，该类 mAP 无意义
    for split in ("train", "val"):
        for cid, cname in enumerate(CLASS_NAMES):
            if stats[split]["cls"][cid] == 0:
                print("[提醒] %s 集里没有 %s 类样本，该类指标不可信" % (split, cname))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
