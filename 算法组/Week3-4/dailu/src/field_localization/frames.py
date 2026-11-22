# -*- coding: utf-8 -*-
"""从录制的视频里取帧，方便标点/调试。

用法：
    python frames.py 视频目录 输出目录 [--count 3]
例：
    python frames.py D:\\videos D:\\videos\\frames --count 3
"""
import argparse
import os

import cv2


def grab_frames(video_path, out_dir, count=3):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[ERROR] 打不开 {video_path}")
        return []
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    stem = os.path.splitext(os.path.basename(video_path))[0]
    print(f"[{stem}] {w}x{h} fps={fps:.1f} frames={total}")

    outs = []
    # 静止视频：随便取中间几帧即可
    ratios = [0.5] if count <= 1 else [0.5 + 0.02 * (i - (count - 1) / 2) for i in range(count)]
    for i, r in enumerate(ratios):
        idx = int(total * r)
        cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, idx))
        ok, frame = cap.read()
        if not ok:
            continue
        name = f"{stem}_{i:02d}.png"
        path = os.path.join(out_dir, name)
        cv2.imwrite(path, frame)
        outs.append(path)
        print(f"   -> {path}  (frame {idx})")
    cap.release()
    return outs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--count", type=int, default=3)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    exts = (".mp4", ".avi", ".mov", ".mkv")
    vids = sorted(f for f in os.listdir(args.video_dir) if f.lower().endswith(exts))
    if not vids:
        print("[ERROR] 目录里没有视频文件")
        return
    print(f"共 {len(vids)} 段视频")
    for v in vids:
        grab_frames(os.path.join(args.video_dir, v), args.out_dir, args.count)


if __name__ == "__main__":
    main()
