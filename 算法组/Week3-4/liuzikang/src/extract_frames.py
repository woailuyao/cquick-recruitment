#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从采集的视频里抽帧，整理成 YOLO 数据集用的图片目录。"""
from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import cv2
import numpy as np


def imwrite_unicode(path: Path, img: np.ndarray, quality: int = 92) -> bool:
    """中文路径也能写的 imwrite。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        return False
    buf.tofile(str(path))
    return True


def hist_similarity(a: np.ndarray, b: np.ndarray) -> float:
    """两张图的 HSV 直方图相似度（0~1，越接近 1 越像）。"""
    ha = cv2.calcHist([a], [0, 1], None, [32, 32], [0, 180, 0, 256])
    hb = cv2.calcHist([b], [0, 1], None, [32, 32], [0, 180, 0, 256])
    cv2.normalize(ha, ha)
    cv2.normalize(hb, hb)
    return float(cv2.compareHist(ha, hb, cv2.HISTCMP_CORREL))


def main() -> int:
    ap = argparse.ArgumentParser(description="从视频抽帧")
    ap.add_argument("--videos", nargs="+", required=True, help="视频路径（支持通配）")
    ap.add_argument("--out", default="dataset/images/all", help="输出图片目录")
    ap.add_argument("--every", type=int, default=15, help="每隔多少帧取一张（默认 15）")
    ap.add_argument("--min-sharp", type=float, default=60.0,
                    help="清晰度下限（拉普拉斯方差），低于此丢弃，默认 60")
    ap.add_argument("--sim-threshold", type=float, default=0.995,
                    help="与上一张的相似度上限，超过则丢弃，默认 0.995")
    ap.add_argument("--max-per-video", type=int, default=0, help="每段最多取多少张，0=不限")
    args = ap.parse_args()

    files: list[str] = []
    for pat in args.videos:
        m = glob.glob(pat)
        files.extend(m if m else [pat])
    files = sorted(f for f in set(files) if Path(f).suffix.lower() in
                   (".mp4", ".avi", ".mov", ".mkv"))
    if not files:
        print("没找到视频文件")
        return 1

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    total_saved = 0
    print(f"输出目录：{out_dir.resolve()}\n")
    for vid in files:
        stem = Path(vid).stem
        short = stem.split("_")[0] + "_" + stem.split("_")[1] if "_" in stem else stem
        cap = cv2.VideoCapture(vid)
        if not cap.isOpened():
            print(f"[跳过] 打不开 {vid}")
            continue
        fps = cap.get(cv2.CAP_PROP_FPS) or 30
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        print(f"[{short}] {Path(vid).name}  {n} 帧  {fps:.1f}fps  "
              f"计划每 {args.every} 帧取 1 张")

        saved = skipped_blur = skipped_sim = 0
        idx = 0
        prev = None
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if idx % args.every == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                if sharp < args.min_sharp:
                    skipped_blur += 1
                elif prev is not None and hist_similarity(prev, hsv) > args.sim_threshold:
                    skipped_sim += 1
                else:
                    name = f"{short}_{idx:06d}.jpg"
                    if imwrite_unicode(out_dir / name, frame):
                        saved += 1
                        prev = hsv
                    if args.max_per_video and saved >= args.max_per_video:
                        break
            idx += 1
        cap.release()
        total_saved += saved
        print(f"        保存 {saved} 张，模糊丢弃 {skipped_blur}，重复丢弃 {skipped_sim}")

    print(f"\n合计 {total_saved} 张 -> {out_dir.resolve()}")
    print("下一步：用 LabelImg 标注（类别名固定写 ball）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
