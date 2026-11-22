#!/usr/bin/env python3
"""从采集视频里抽帧，用于标注。

建议：每隔 10~15 帧抽一张（30fps 下约 0.3~0.5 秒），相邻帧太像没有意义。

用法：
  python tools/extract_frames.py --video raw/ball_v01.mp4 --out raw/frames/ball_v01 --every 12
  python tools/extract_frames.py --video raw/ball_v01.mp4 --out raw/frames/ball_v01 --every 12 --start 60 --max 60
"""
from __future__ import annotations

import argparse
import os

import cv2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--every", type=int, default=12, help="每多少帧抽一张")
    ap.add_argument("--start", type=int, default=0, help="从第几帧开始")
    ap.add_argument("--max", type=int, default=0, help="最多抽多少张，0=不限")
    ap.add_argument("--quality", type=int, default=95, help="jpg 质量")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"[error] 打不开视频 {args.video}")
        return 2

    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[video] {args.video}  {w}x{h}  fps={fps:.2f}  frames={total}")

    os.makedirs(args.out, exist_ok=True)
    stem = os.path.splitext(os.path.basename(args.video))[0]
    idx = 0
    saved = 0
    cap.set(cv2.CAP_PROP_POS_FRAMES, args.start)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.every == 0:
            name = os.path.join(args.out, f"{stem}_{idx:05d}.jpg")
            cv2.imwrite(name, frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
            saved += 1
            if saved % 20 == 0:
                print(f"  ... 已保存 {saved} 张 (frame {idx})")
            if args.max and saved >= args.max:
                break
        idx += 1

    cap.release()
    print(f"[done] 抽帧 {saved} 张 -> {args.out}")
    if saved and args.every < 8:
        print("[warn] --every 太小，抽出来的相邻帧几乎一样，"
              "训练集会出现近重复样本；建议 >= 10")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
