# -*- coding: utf-8 -*-
"""extract_fixed_frames.py —— 从固定外参重拍视频抽帧，供人工标注 L/T。

抽帧原则：
  * 按帧号等间隔抽，不按时间随机抽；相邻帧至少隔 1 秒，避免近重复图。
  * 跳过开头/结尾若干帧（手抖、收尾），只取稳定段。
  * 文件名保留视频前缀：M10_00040.jpg → prepare_landmark_dataset.py 能按
    整段视频划分 train/val，不会把同一段视频的近重复帧拆到两边。
  * 只写图片和 classes.txt / manifest.csv，**不写任何 txt 标签**，
    由人工在 labelImg 里标注。重复运行时不会覆盖已有标签。

用法（Windows conda cv_practice）：
  python extract_fixed_frames.py
  python extract_fixed_frames.py --videos M10 M14 --step 30
"""
import argparse
import csv
import os

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=r"D:\captures")
    ap.add_argument("--out", default=r"D:\captures\_anno\fixed_work")
    ap.add_argument("--videos", nargs="+",
                    default=["M9", "M10", "M11", "M12", "M13", "M14", "M15"])
    ap.add_argument("--start", type=int, default=40,
                    help="跳过开头多少帧（约 2 秒）")
    ap.add_argument("--step", type=int, default=20,
                    help="抽帧间隔（约 1 秒）")
    ap.add_argument("--tail", type=int, default=20,
                    help="跳过结尾多少帧")
    ap.add_argument("--val-videos", nargs="*", default=["M11"],
                    help="建议留作验证集的整段视频")
    ap.add_argument("--force", action="store_true", help="覆盖已存在的图片")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "classes.txt"), "w",
              encoding="utf-8") as f:
        f.write("L\nT\n")

    val = set(args.val_videos)
    manifest = []
    total = 0
    for name in args.videos:
        path = os.path.join(args.src, name + ".mp4")
        if not os.path.exists(path):
            print("[跳过] 找不到", path)
            continue
        cap = cv2.VideoCapture(path)
        fps = cap.get(cv2.CAP_PROP_FPS) or 21.0
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        indices = list(range(args.start, max(args.start, n - args.tail),
                             args.step))
        got = 0
        for idx in indices:
            dst = os.path.join(args.out, "%s_%05d.jpg" % (name, idx))
            if os.path.exists(dst) and not args.force:
                got += 1
                continue
            cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, frame = cap.read()
            if not ok:
                continue
            cv2.imwrite(dst, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            got += 1
            manifest.append({
                "file": os.path.basename(dst),
                "video": name,
                "frame": idx,
                "time_sec": round(idx / fps, 2),
                "suggested_split": "val" if name in val else "train",
            })
        cap.release()
        total += got
        print("%s: %d 帧 (%d..%d, step %d)  建议 %s"
              % (name, got, args.start, indices[-1] if indices else args.start,
                 args.step, "val" if name in val else "train"))

    # 即使图片已存在（重跑），manifest 也要完整写一遍
    if not manifest:
        for name in args.videos:
            path = os.path.join(args.src, name + ".mp4")
            if not os.path.exists(path):
                continue
            cap = cv2.VideoCapture(path)
            fps = cap.get(cv2.CAP_PROP_FPS) or 21.0
            n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            for idx in range(args.start, max(args.start, n - args.tail),
                             args.step):
                fn = "%s_%05d.jpg" % (name, idx)
                if not os.path.exists(os.path.join(args.out, fn)):
                    continue
                manifest.append({
                    "file": fn, "video": name, "frame": idx,
                    "time_sec": round(idx / fps, 2),
                    "suggested_split": "val" if name in val else "train",
                })
            cap.release()

    mpath = os.path.join(args.out, "manifest.csv")
    with open(mpath, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["file", "video", "frame",
                                          "time_sec", "suggested_split"])
        w.writeheader()
        w.writerows(manifest)

    note = os.path.join(args.out, "标注说明.txt")
    with open(note, "w", encoding="utf-8") as f:
        f.write("固定外参重拍视频抽帧 —— L/T 标注说明\n")
        f.write("=" * 40 + "\n")
        f.write("图片：%d 张（跳过开头 %.1fs / 结尾 %.1fs，间隔 %d 帧）\n"
                % (total, args.start / 21.0, args.tail / 21.0, args.step))
        f.write("类别：0=L，1=T（classes.txt 只有两行，不要加类别）\n")
        f.write("只标场地上的 L/T 标志本体；白线拐角、球门网、门框、"
                "桌椅不要画框（历次误检就来自这些）\n")
        f.write("建议：M11 整段留验证集，其余进训练集；见 manifest.csv\n")
        f.write("标注完不要改文件名；不要删除 classes.txt / manifest.csv\n")
        f.write("\n启动标注（WSL Ubuntu-22.04）：\n")
        f.write("  bash /home/weikai/vision_ws/k1_soccer_vision/anno_landmark.sh "
                "/mnt/d/captures/_anno/fixed_work\n")

    print("\n完成：%d 张图片 → %s" % (total, args.out))
    print("清单：%s" % mpath)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
