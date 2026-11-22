#!/usr/bin/env python3
"""从抽出来的帧里挑"互不相似"的图，去掉几乎重复的帧；标签一起搬过去。

细则要求"同一段视频连续抽取的高度相似帧不能被当作大量独立样本"，
所以流程是：先密抽（--every 5），再用这个工具筛掉相似帧。

单段：
  python tools\\pick_diverse.py --src C:\\...\\frames\\ball_v04 --out C:\\...\\frames_sel\\ball_v04
全部（--src 给的是"装着各段文件夹的根目录"时自动逐段处理）：
  python tools\\pick_diverse.py --src C:\\...\\raw\\frames --out C:\\...\\raw\\frames_sel
"""
from __future__ import annotations

import argparse
import os
import shutil

import cv2
import numpy as np

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")


def descriptor(path: str):
    im = cv2.imread(path)
    if im is None:
        return None
    small = cv2.resize(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (32, 18),
                       interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [12, 8], [0, 180, 0, 256])
    cv2.normalize(hist, hist)
    return small.ravel(), hist.ravel()


def dist(d1, d2) -> float:
    a1, h1 = d1
    a2, h2 = d2
    d_gray = float(np.mean(np.abs(a1 - a2)))
    d_hist = float(cv2.compareHist(h1.reshape(-1, 1), h2.reshape(-1, 1),
                                   cv2.HISTCMP_BHATTACHARYYA))
    return 0.7 * d_gray + 0.3 * d_hist


def process_one(src: str, out: str, max_n: int, thresh: float, dry: bool) -> dict:
    files = sorted(f for f in os.listdir(src) if f.lower().endswith(IMG_EXT))
    name = os.path.basename(os.path.normpath(src))
    if not files:
        print(f"[pick] {name}: 没有图片，跳过")
        return {}

    kept = []
    for f in files:
        d = descriptor(os.path.join(src, f))
        if d is None:
            continue
        if not kept:
            kept.append((f, d))
            continue
        if min(dist(d, k[1]) for k in kept) >= thresh:
            kept.append((f, d))
        if max_n and len(kept) >= max_n:
            break

    if not dry:
        os.makedirs(out, exist_ok=True)
        n_lbl = 0
        for f, _ in kept:
            shutil.copy2(os.path.join(src, f), os.path.join(out, f))
            stem = os.path.splitext(f)[0]
            src_lbl = os.path.join(src, stem + ".txt")
            if os.path.exists(src_lbl):
                shutil.copy2(src_lbl, os.path.join(out, stem + ".txt"))
                n_lbl += 1
        print(f"[pick] {name:12s} {len(files):3d} 张 -> 保留 {len(kept):3d} 张"
              f"（带去 {n_lbl} 个标签）")
    else:
        print(f"[pick] {name:12s} {len(files):3d} 张 -> 会保留 {len(kept):3d} 张")
        n_lbl = 0
    return {"name": name, "total": len(files), "kept": len(kept), "lbl": n_lbl}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="单段文件夹，或装着多段的根目录")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max", type=int, default=0, help="每段最多保留多少张，0=不限")
    ap.add_argument("--thresh", type=float, default=0.030,
                    help="与已保留图的最小差异；越大留得越少")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    src = os.path.abspath(args.src)
    out = os.path.abspath(args.out)
    if not os.path.isdir(src):
        print(f"[error] 不存在：{src}")
        return 2

    has_img = any(f.lower().endswith(IMG_EXT) for f in os.listdir(src))
    subs = [d for d in sorted(os.listdir(src))
            if os.path.isdir(os.path.join(src, d))]

    print(f"[pick] 阈值 {args.thresh}，最多每段 {args.max or '不限'} 张\n")
    results = []
    if has_img:
        r = process_one(src, out, args.max, args.thresh, args.dry_run)
        if r:
            results.append(r)
    elif subs:
        for d in subs:
            r = process_one(os.path.join(src, d), os.path.join(out, d),
                            args.max, args.thresh, args.dry_run)
            if r:
                results.append(r)
    else:
        print(f"[error] {src} 里既没有图片也没有子文件夹")
        return 2

    if results:
        tot_in = sum(r["total"] for r in results)
        tot_out = sum(r["kept"] for r in results)
        print(f"\n[汇总] {tot_in} 张 -> {tot_out} 张，去掉 "
              f"{tot_in - tot_out} 张相似帧 "
              f"({100.0*(tot_in-tot_out)/max(tot_in,1):.0f}%)")
        if not args.dry_run:
            print(f"[汇总] 输出：{out}")
        else:
            print("[汇总] dry-run，没有拷贝文件")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
