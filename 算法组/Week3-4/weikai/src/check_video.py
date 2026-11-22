#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
check_video.py — 录好的视频质检（采完素材回来先用它过一遍）

检查项：
  1. 分辨率、帧数、实际平均帧率（按时间戳算，不看容器标称值）
  2. 花屏/截断：逐帧找"从某行起整行全黑"的位置——USB/IP 截断帧的典型特征
  3. 过曝/黑屏：整帧平均亮度异常、全黑帧
  4. 运动程度：帧间差分均值，太小的说明镜头几乎没动（不够多样性）

用法：
  python check_video.py V1.mp4 V2.mp4
  python check_video.py --dir D:\\captures
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np


def check(path, sample_step=5):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        return {"file": path, "error": "无法打开（编码不支持或文件损坏）"}

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    declared_fps = cap.get(cv2.CAP_PROP_FPS)

    n = 0
    truncated = 0
    black_frames = 0
    brights = []
    diffs = []
    prev = None
    first_black_row = None
    t0 = None
    t_end = None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if t0 is None:
            t0 = cap.get(cv2.CAP_PROP_POS_MSEC)
        t_end = cap.get(cv2.CAP_PROP_POS_MSEC)
        if n % sample_step == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            brights.append(float(gray.mean()))
            # 整行几乎全黑的行
            row_dark = (gray < 8).mean(axis=1)
            fb = np.where(row_dark > 0.99)[0]
            if len(fb) > h * 0.05:  # 超过 5% 的行全黑 -> 判为截断
                truncated += 1
                if first_black_row is None:
                    first_black_row = int(fb[0])
            if gray.mean() < 3:
                black_frames += 1
            if prev is not None:
                diffs.append(float(cv2.absdiff(gray, prev).mean()))
            prev = gray
        n += 1
    cap.release()

    dur = (t_end - t0) / 1000.0 if (t0 is not None and t_end is not None) else 0
    real_fps = n / dur if dur > 0 else 0
    return {
        "file": os.path.basename(path),
        "size_mb": round(os.path.getsize(path) / 1024 / 1024, 2),
        "res": f"{w}x{h}",
        "frames": n,
        "duration_s": round(dur, 2),
        "container_fps": round(declared_fps, 2),
        "real_fps": round(real_fps, 2),
        "truncated_samples": truncated,
        "first_black_row": first_black_row,
        "black_frames": black_frames,
        "mean_brightness": round(float(np.mean(brights)), 1) if brights else None,
        "motion": round(float(np.mean(diffs)), 2) if diffs else None,
    }


def verdict(r):
    if "error" in r:
        return "✗ " + r["error"]
    issues = []
    if r["truncated_samples"] > 0:
        issues.append(f"有 {r['truncated_samples']} 帧出现截断性黑区（首黑行 {r['first_black_row']}）")
    if r["black_frames"]:
        issues.append(f"{r['black_frames']} 个全黑帧")
    if r["motion"] is not None and r["motion"] < 0.8:
        issues.append("画面几乎没有变化（运动量过低）")
    if r["mean_brightness"] is not None and (r["mean_brightness"] < 15 or r["mean_brightness"] > 235):
        issues.append("整体过暗或过曝")
    return "✓ 正常" if not issues else "⚠ " + "；".join(issues)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*")
    ap.add_argument("--dir", default=None)
    ap.add_argument("--step", type=int, default=5, help="每 N 帧抽一帧检查")
    args = ap.parse_args()

    files = list(args.files)
    if args.dir:
        for ext in ("*.mp4", "*.avi", "*.mkv", "*.mov"):
            files += sorted(glob.glob(os.path.join(args.dir, ext)))
    if not files:
        print("没有指定文件。用法: python check_video.py V1.mp4  或  --dir <目录>")
        sys.exit(1)

    for f in files:
        r = check(f, args.step)
        print("=" * 62)
        print("文件:", r.get("file"))
        if "error" in r:
            print(verdict(r))
            continue
        print(f"  大小 {r['size_mb']} MB | 分辨率 {r['res']} | 帧数 {r['frames']} | "
              f"时长 {r['duration_s']} s")
        print(f"  容器标称 {r['container_fps']} fps | 实测 {r['real_fps']} fps")
        print(f"  平均亮度 {r['mean_brightness']} | 运动量 {r['motion']} | "
              f"截断样本 {r['truncated_samples']}")
        print("  判定:", verdict(r))


if __name__ == "__main__":
    main()
