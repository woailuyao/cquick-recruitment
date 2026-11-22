"""抽帧（去模糊 + 去重 + 均匀取样），用于补充标注数据。

比旧的 extract_frames.py 多做两件事：
  1. 自动估算清晰度阈值（按这条视频自己的分位数，不是固定 40）
  2. 相邻帧几乎一样时只留一张（按 80x45 缩略图差异判重），避免"同一帧标 10 遍"

    python scripts/extract_frames_v2.py ^
        --videos F:\video\T1.avi F:\video\X1.avi ^
        --output dataset/images/batch_20261121 --count 70

输出文件名保持 <视频名>_<帧号>.jpg，方便 audit_dataset.py 按视频段分组。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))


def sharpness(gray):
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def signature(gray):
    return cv2.resize(gray, (80, 45), interpolation=cv2.INTER_AREA).astype(np.float32)


def parse_segments(spec):
    """['0:960', '1260:1560'] -> [(0, 960), (1260, 1560)]"""
    segs = []
    for s in spec or []:
        a, b = s.split(":")
        segs.append((int(a), int(b)))
    return segs


def inside(i, segs):
    return not segs or any(a <= i < b for a, b in segs)


def scan(video, step=2, segs=()):
    """第一遍：采样计算清晰度分布 + 记下每帧缩略图。"""
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise SystemExit(f"打不开 {video}")
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    rows = []
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % step == 0 and inside(i, segs):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            rows.append((i, sharpness(gray), signature(gray)))
        i += 1
    cap.release()
    return rows, n, fps


def choose(rows, count, sharp_pct, min_gap, dup_thresh):
    sharp = np.array([r[1] for r in rows])
    thr = float(np.percentile(sharp, sharp_pct))
    picked = []
    last_sig = None
    last_i = -10 ** 9
    for i, s, sig in rows:
        if s < thr:
            continue
        if last_sig is not None and i - last_i < min_gap:
            continue
        if last_sig is not None and float(np.abs(sig - last_sig).mean()) < dup_thresh:
            continue
        picked.append((i, s))
        last_sig = sig
        last_i = i
    if len(picked) > count:
        idx = np.linspace(0, len(picked) - 1, count).round().astype(int)
        picked = [picked[k] for k in dict.fromkeys(idx)]
    return picked, thr, sharp


def save(video, picked, out_dir, stem):
    cap = cv2.VideoCapture(str(video))
    want = {i: s for i, s in picked}
    saved = []
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i in want:
            p = out_dir / f"{stem}_{i:06d}.jpg"
            cv2.imwrite(str(p), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            saved.append((p, want[i]))
        i += 1
    cap.release()
    return saved


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", nargs="+", required=True)
    ap.add_argument("--output", default=str(ROOT / "dataset" / "images" / "batch_new"))
    ap.add_argument("--count", type=int, default=70, help="每条视频最多保留多少张")
    ap.add_argument("--sharp-percentile", type=float, default=55.0,
                    help="清晰度阈值取该分位数（越大越挑剔）")
    ap.add_argument("--min-gap", type=int, default=12, help="两张之间至少隔多少帧")
    ap.add_argument("--dup-thresh", type=float, default=4.0,
                    help="缩略图平均绝对差小于该值视为重复")
    ap.add_argument("--sheet", default=None, help="输出预览拼图路径")
    ap.add_argument("--segments", nargs="*", default=None,
                    help="只在这些帧区间抽帧，例如 0:960 1260:1560（不同视频不同区间就分两次跑）")
    args = ap.parse_args()

    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    segs = parse_segments(args.segments)
    if segs:
        print(f"只抽取区间: {segs}")

    all_saved = []
    for v in args.videos:
        v = Path(v)
        print(f"\n=== {v.name} ===")
        rows, n, fps = scan(v, segs=segs)
        if not rows:
            print("  没有读到帧，跳过")
            continue
        picked, thr, sharp = choose(rows, args.count, args.sharp_percentile,
                                    args.min_gap, args.dup_thresh)
        print(f"  总帧数 {n}  时长 {n/max(fps,1):.1f}s  fps {fps:.1f}")
        print(f"  清晰度(Laplacian var): 中位 {np.median(sharp):.0f}  "
              f"阈值 {thr:.0f} (p{args.sharp_percentile:.0f})")
        saved = save(v, picked, out, v.stem)
        print(f"  保存 {len(saved)} 张 -> {out}")
        all_saved += saved

    if not all_saved:
        raise SystemExit("一张都没保存")

    print(f"\n合计 {len(all_saved)} 张")
    sh = np.array([s for _, s in all_saved])
    print(f"清晰度: 最低 {sh.min():.0f}  中位 {np.median(sh):.0f}  最高 {sh.max():.0f}")

    # 预览拼图
    if args.sheet:
        cols, tw = 6, 320
        thumbs = []
        for p, s in all_saved:
            im = cv2.imread(str(p))
            th = int(tw * im.shape[0] / im.shape[1])
            t = cv2.resize(im, (tw, th))
            cv2.putText(t, f"{p.stem}  sh={s:.0f}", (4, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
            thumbs.append(t)
        rows_n = (len(thumbs) + cols - 1) // cols
        h = thumbs[0].shape[0]
        sheet = np.zeros((rows_n * h, cols * tw, 3), np.uint8)
        for k, t in enumerate(thumbs):
            r, c = divmod(k, cols)
            sheet[r * h:r * h + t.shape[0], c * tw:c * tw + t.shape[1]] = t
        cv2.imwrite(str(args.sheet), sheet, [cv2.IMWRITE_JPEG_QUALITY, 88])
        print(f"预览拼图: {args.sheet}")

    print("\n下一步：")
    print(f"  python scripts\\label_tool_markers.py --images {args.output}")
    print("  标完把 jpg+txt 一起移到 dataset\\images\\selected\\ 和 dataset\\labels\\selected\\")


if __name__ == "__main__":
    main()