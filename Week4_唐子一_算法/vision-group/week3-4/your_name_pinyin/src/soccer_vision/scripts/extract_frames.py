#!/usr/bin/env python3
"""从原始采集视频按固定间隔抽帧，整理为 YOLO 数据集目录。

关键点（考核要求）：
  - 数据集按【独立视频片段】划分 train/val/test，不能把同一段视频的
    相邻帧随机打散（否则指标虚高 / 数据泄漏）。
  - 抽帧时记录视频来源、抽帧间隔、日期，便于 dataset_summary.md 汇总。

用法：
    python extract_frames.py --videos v1.mp4 v2.mp4 v3.mp4 v4.mp4 v5.mp4 \
        --out soccer_dataset --interval 5 --train 0.7 --val 0.2
"""

import argparse
import os
import shutil
from pathlib import Path

import cv2


def extract_one(video: str, out_dir: str, interval: int) -> int:
    """抽一段视频的帧，返回抽帧数量。"""
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        print(f"[FAIL] 无法打开 {video}")
        return 0
    name = Path(video).stem
    idx = 0
    saved = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % interval == 0:
            out_path = os.path.join(out_dir, f"{name}_{saved:06d}.jpg")
            cv2.imwrite(out_path, frame)
            saved += 1
        idx += 1
    cap.release()
    print(f"  {video}: 抽帧 {saved} 张")
    return saved


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", nargs="+", required=True, help="原始视频列表（>=5段）")
    ap.add_argument("--out", default="soccer_dataset")
    ap.add_argument("--interval", type=int, default=5, help="每 interval 帧抽一张")
    ap.add_argument("--train", type=float, default=0.7)
    ap.add_argument("--val", type=float, default=0.2)
    args = ap.parse_args()

    if len(args.videos) < 5:
        print("[WARN] 考核要求至少 5 段独立视频")

    # 先按视频抽帧到临时目录
    tmp = os.path.join(args.out, "_tmp_all")
    os.makedirs(tmp, exist_ok=True)
    for v in args.videos:
        extract_one(v, tmp, args.interval)

    files = sorted(Path(tmp).glob("*.jpg"))
    n = len(files)
    print(f"共抽帧 {n} 张")

    # 按视频片段划分（文件名前缀 = 视频名）
    by_video = {}
    for f in files:
        stem = f.stem.rsplit("_", 1)[0]  # 去掉 _000000 后缀
        by_video.setdefault(stem, []).append(f)

    videos = list(by_video.keys())
    n_v = len(videos)
    n_train = max(1, int(n_v * args.train))
    n_val = max(1, int(n_v * args.val))
    train_vids = videos[:n_train]
    val_vids = videos[n_train:n_train + n_val]
    test_vids = videos[n_train + n_val:]

    # 如果 test 为空，把最后一段视频划给 test
    if not test_vids and val_vids:
        test_vids = [val_vids[-1]]
        val_vids = val_vids[:-1]

    for split, vids in [("train", train_vids), ("val", val_vids), ("test", test_vids)]:
        img_dir = os.path.join(args.out, "images", split)
        lbl_dir = os.path.join(args.out, "labels", split)
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(lbl_dir, exist_ok=True)
        for v in vids:
            for f in by_video[v]:
                shutil.copy(f, os.path.join(img_dir, f.name))
                # 生成空标签占位（后续标注时覆盖）
                open(os.path.join(lbl_dir, f.with_suffix(".txt").name), "a").close()

    shutil.rmtree(tmp, ignore_errors=True)

    print("划分结果：")
    print(f"  train: {len(train_vids)} 段视频 ({train_vids})")
    print(f"  val:   {len(val_vids)} 段视频 ({val_vids})")
    print(f"  test:  {len(test_vids)} 段视频 ({test_vids})")


if __name__ == "__main__":
    main()
