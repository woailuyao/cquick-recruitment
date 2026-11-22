#!/usr/bin/env python3
"""一条命令抽完所有视频的帧（内部自己循环，不用你在命令行里写 for）。

用法（就一行，别和别的命令粘一起）：
  python tools\\extract_all.py --videos C:\\Users\\HUAWEI\\Desktop\\week34\\raw\\*.avi --out-root C:\\Users\\HUAWEI\\Desktop\\week34\\raw\\frames --every 15
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import cv2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", required=True,
                    help="视频路径，可以用通配符，如 raw\\*.avi")
    ap.add_argument("--out-root", required=True, help="每段视频输出到 输出目录\\视频名\\")
    ap.add_argument("--every", type=int, default=15, help="每多少帧抽一张")
    ap.add_argument("--quality", type=int, default=95)
    args = ap.parse_args()

    pattern = args.videos
    files = sorted(glob.glob(pattern))
    if not files:
        # 通配符没被展开时再试一次（cmd 默认不展开通配符）
        d = os.path.dirname(pattern) or "."
        files = sorted(p for p in glob.glob(os.path.join(d, "*"))
                       if os.path.splitext(p)[1].lower() in (".avi", ".mp4", ".mov", ".mkv"))
    if not files:
        print(f"[error] 没找到视频：{pattern}")
        return 2

    print(f"[batch] 找到 {len(files)} 段视频，输出到 {args.out_root}\n")
    total_saved = 0
    summary = []

    for path in files:
        name = os.path.splitext(os.path.basename(path))[0]
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            print(f"[skip] 打不开 {path}")
            continue
        fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
        n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        out_dir = os.path.join(args.out_root, name)
        os.makedirs(out_dir, exist_ok=True)

        idx = 0
        saved = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if idx % args.every == 0:
                fn = os.path.join(out_dir, f"{name}_{idx:05d}.jpg")
                cv2.imwrite(fn, frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
                saved += 1
            idx += 1
        cap.release()

        total_saved += saved
        summary.append((name, w, h, fps, n_total, saved))
        print(f"[done] {name:14s} {w}x{h} {n_total:5d}帧 {fps:5.1f}fps "
              f"-> 抽 {saved:3d} 张  {out_dir}")

    print("\n=== 汇总 ===")
    for name, w, h, fps, n, saved in summary:
        print(f"  {name:14s} {w}x{h}  视频{n:5d}帧  抽出{saved:3d}张")
    print(f"  合计抽出 {total_saved} 张")
    if total_saved == 0:
        print("[error] 一张都没抽出来，检查 --videos 路径")
        return 3
    print("\n下一步：用 labelImg 打开这些文件夹标注")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
