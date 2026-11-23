# -*- coding: utf-8 -*-
"""
标志段质量筛查：给每段标志视频打分，决定哪些段值得抽帧

为什么要筛
    L_3 这类段整体发虚过曝，白线对比度低，标出来的框对训练是噪声。
    与其全量抽 1750 张再手工删，不如先按客观指标把烂段剔掉。

指标
    sharp   拉普拉斯方差（越大越清晰，<120 基本是糊的）
    white   顶帽变换后的"细亮结构"占比（%）。用顶帽而不是"低饱和+高亮"，
            因为这批素材整幅都偏灰（饱和度中位数仅 39~56），
            "低饱和+高亮"会把白墙和过曝天空一起算进去，完全失去判别力。
    over    过曝像素占比（V>=245，%）
    green   绿色草皮像素占比（%）

用法
    python screen_mark_videos.py
    python screen_mark_videos.py --min-sharp 120
"""
import argparse
import glob
import os

import cv2
import numpy as np

VIDEO_DIR = "videos"
PREFIXES = ("L", "T", "X", "neg")
SAMPLE_EVERY = 15          # 每段均匀采 15 帧评估


def score_frame(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    # 顶帽：只保留比周围亮、且比 15x15 邻域窄的细结构 —— 正好是白线
    top = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, np.ones((15, 15), np.uint8))
    white = float((top >= 40).mean() * 100)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    over = float((v >= 245).mean() * 100)
    green = float(((h >= 30) & (h <= 100) & (s >= 25) &
                   (v >= 40) & (v <= 200)).mean() * 100)
    return dict(sharp=sharp, white=white, over=over, green=green,
                mean_v=float(v.mean()))


def scan(path, nth=SAMPLE_EVERY, min_sharp=120.0):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        return None
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
    if n <= 0:
        cap.release()
        return None
    idxs = np.linspace(0, n - 1, min(nth, n)).astype(int)
    rows = []
    for i in idxs:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, fr = cap.read()
        if not ok:
            continue
        rows.append(score_frame(fr))
    cap.release()
    if not rows:
        return None

    def med(k):
        return float(np.median([r[k] for r in rows]))

    return dict(name=os.path.splitext(os.path.basename(path))[0],
                dur=n / fps, n=n,
                sharp=med("sharp"), white=med("white"),
                over=med("over"), green=med("green"), mean_v=med("mean_v"),
                good=sum(1 for r in rows if verdict(r, min_sharp) == "OK"),
                n_samples=len(rows))


def verdict(r, min_sharp=120.0, max_over=20.0, min_white=1.0, min_green=35.0):
    bad = []
    if r["sharp"] < min_sharp:
        bad.append("糊")
    if r["over"] > max_over:
        bad.append("过曝")
    if r["white"] < min_white:
        bad.append("无线")
    if r["green"] < min_green:
        bad.append("不像场地")
    return "OK" if not bad else "/".join(bad)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-sharp", type=float, default=120.0)
    ap.add_argument("--src", default=VIDEO_DIR)
    ap.add_argument("--samples", type=int, default=SAMPLE_EVERY)
    args = ap.parse_args()

    files = []
    for p in PREFIXES:
        files += sorted(glob.glob(os.path.join(args.src, f"{p}_*.mp4")),
                        key=lambda s: int("".join(c for c in os.path.basename(s).split("_")[1]
                                                  if c.isdigit()) or 0))
    if not files:
        print("没找到标志视频")
        return

    print(f"{'段':<10} {'时长':>6} {'sharp':>7} {'white%':>7} {'over%':>6} "
          f"{'green%':>7} {'好帧':>7}  判定")
    print("-" * 68)
    keep, drop = [], []
    for f in files:
        r = scan(f, args.samples, args.min_sharp)
        if r is None:
            print(f"{os.path.basename(f):<10} 读取失败")
            continue
        v = verdict(r, args.min_sharp)
        flag = "" if v == "OK" else "  <<<"
        print(f"{r['name']:<10} {r['dur']:6.1f} {r['sharp']:7.1f} {r['white']:7.2f} "
              f"{r['over']:6.1f} {r['green']:7.1f} {r['good']:3d}/{r['n_samples']:<3d} {v}{flag}")
        (keep if v == "OK" else drop).append(r["name"])

    print("-" * 62)
    print(f"合格 {len(keep)} 段：{' '.join(keep)}")
    print(f"剔除 {len(drop)} 段：{' '.join(drop) if drop else '（无）'}")
    if keep:
        with open("mark_keep.txt", "w", encoding="utf-8") as f:
            f.write("\n".join(keep) + "\n")
        print("已写 mark_keep.txt（extract_mark_frames.py 会读它）")


if __name__ == "__main__":
    main()
