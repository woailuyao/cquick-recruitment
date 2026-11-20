#!/usr/bin/env python3
"""Extract annotation frames from task-2 field-marker videos.

Reads every video in --videos (files or a directory), samples frames at a
fixed time interval, drops near-duplicate and blurred frames, and writes
JPEGs named <video_stem>_f<frame_index>.jpg plus a manifest.json that maps
each frame back to its source video (needed for leakage-free splitting).

Usage:
    python3 extract_frames.py --videos ./videos --out ./frames --target-total 320
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

VIDEO_EXTS = {".avi", ".mp4", ".mov", ".mkv"}


def frame_sharpness(gray):
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def extract_from_video(path, out_dir, quota, interval_frames, diff_thresh,
                       min_sharpness):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        print(f"warning: cannot open {path}")
        return []
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    kept = []
    last_small = None
    idx = 0
    while len(kept) < quota:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, (160, 90))
        keep = True
        if last_small is not None:
            diff = float(np.mean(cv2.absdiff(small, last_small)))
            if diff < diff_thresh:
                keep = False
        if keep and frame_sharpness(gray) < min_sharpness:
            keep = False
        if keep:
            name = f"{path.stem}_f{idx:06d}.jpg"
            cv2.imwrite(str(out_dir / name), frame)
            kept.append({"frame": name, "video": path.name,
                         "frame_index": idx})
            last_small = small
        idx += interval_frames
        if idx >= total > 0:
            break
    cap.release()
    return kept


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", required=True,
                    help="a video file or a directory of videos")
    ap.add_argument("--out", default="frames")
    ap.add_argument("--target-total", type=int, default=320,
                    help="approximate total frames across all videos")
    ap.add_argument("--interval-sec", type=float, default=0.5,
                    help="minimum seconds between sampled frames")
    ap.add_argument("--diff-thresh", type=float, default=2.0,
                    help="mean abs diff below which a frame is a duplicate")
    ap.add_argument("--min-sharpness", type=float, default=30.0,
                    help="Laplacian variance floor; lower keeps softer frames")
    args = ap.parse_args()

    src = Path(args.videos)
    if src.is_dir():
        videos = sorted(p for p in src.iterdir()
                        if p.suffix.lower() in VIDEO_EXTS)
    else:
        videos = [src]
    if not videos:
        raise SystemExit(f"no videos found at {args.videos}")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Probe durations so the frame quota is shared proportionally.
    durations = {}
    for v in videos:
        cap = cv2.VideoCapture(str(v))
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        count = cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0
        cap.release()
        durations[v] = max(count / fps, 1.0)
    total_dur = sum(durations.values())

    manifest = []
    for v in videos:
        cap = cv2.VideoCapture(str(v))
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        cap.release()
        quota = max(10, round(args.target_total * durations[v] / total_dur))
        interval = max(1, round(fps * args.interval_sec))
        got = extract_from_video(v, out_dir, quota, interval,
                                 args.diff_thresh, args.min_sharpness)
        print(f"{v.name}: kept {len(got)} frames (quota {quota})")
        manifest.extend(got)

    (out_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"total: {len(manifest)} frames from {len(videos)} videos -> {out_dir}")
    if len(videos) < 5:
        print("warning: spec asks for >= 5 independent videos")


if __name__ == "__main__":
    main()
