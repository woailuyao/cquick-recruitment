# -*- coding: utf-8 -*-
"""抽帧：从实拍视频里按「时间间隔 + 清晰度 + 去重」筛选训练帧。

为什么要这样抽（细则 1.3）：
    同一段视频的连续相邻帧高度相似，若全部入集，等于把同一张图复制几十份，
    指标会虚高、且违反「高度相似帧不算独立样本」。

策略：
    1. 每视频按固定时间间隔取样（默认 1.5 s）；
    2. 丢弃模糊帧（中心区拉普拉斯方差 < 阈值）；
    3. 与「上一张保留帧」做灰度差，过于相似则跳过；
    4. 每段视频限制上限，保证五段视频分布均衡。

用法：
    python tools/frame_extract.py
    python tools/frame_extract.py --every 1.0 --max-per-video 80 --min-sharp 60
"""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import DATASET_DIR, rel                     # noqa: E402

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                                  # noqa: BLE001
    pass

VIDEO_EXTS = {".mp4", ".avi", ".mov", ".mkv"}


def sharpness(gray: np.ndarray) -> float:
    h, w = gray.shape[:2]
    return float(cv2.Laplacian(gray[int(h * .2):int(h * .8), int(w * .2):int(w * .8)],
                               cv2.CV_64F).var())


def diff_score(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.abs(a.astype(np.float32) - b.astype(np.float32)).mean())


def extract_one(video: Path, out_dir: Path, every: float, min_sharp: float,
                min_diff: float, max_keep: int, adaptive_ratio: float = 0.45) -> dict:
    """两阶段抽取：先按间隔取样并测清晰度，再用「该段自身的中位数」定阈值。

    绝对阈值在不同场景下不可比：草皮场地中位约 37，白墙地板可能 200+。
    用相对阈值（median × ratio）配一个绝对下限，既不会误杀低纹理场景，
    也能挡住真正模糊的帧。
    """
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        return {"video": video.name, "total": 0, "kept": 0, "err": "打不开"}
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, int(round(fps * every)))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0

    # 阶段一：按间隔取样
    samples = []          # (sharpness, frame, gray)
    idx = 0
    while True:
        ok = cap.grab()
        if not ok:
            break
        if idx % step == 0:
            ok, frame = cap.retrieve()
            if ok and frame is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                samples.append((sharpness(gray), frame, gray))
        idx += 1
    cap.release()

    if not samples:
        return {"video": video.name, "total": total, "kept": 0,
                "blur": 0, "dup": 0, "err": "未取到帧"}

    # 阶段二：相对阈值 + 去重 + 上限
    sharp_list = np.array([s[0] for s in samples])
    thr = max(min_sharp, adaptive_ratio * float(np.median(sharp_list)))

    out_dir.mkdir(parents=True, exist_ok=True)
    kept, skipped_blur, skipped_dup = 0, 0, 0
    prev_gray = None
    for sharp, frame, gray in samples:
        if kept >= max_keep:
            break
        if sharp < thr:
            skipped_blur += 1
            continue
        if prev_gray is not None and diff_score(gray, prev_gray) < min_diff:
            skipped_dup += 1
            continue
        kept += 1
        cv2.imwrite(str(out_dir / f"frame_{kept:04d}.png"), frame)
        prev_gray = gray

    return {"video": video.name, "total": total, "kept": kept, "blur": skipped_blur,
            "dup": skipped_dup, "sampled": len(samples), "thr": float(thr), "err": ""}


def main() -> int:
    ap = argparse.ArgumentParser(description="从视频抽帧")
    ap.add_argument("--videos", default="data/videos")
    ap.add_argument("--out", default="dataset/raw")
    ap.add_argument("--every", type=float, default=1.5, help="采样间隔（秒）")
    ap.add_argument("--min-sharp", type=float, default=12.0,
                    help="绝对下限；实际阈值 = max(该值, 该段中位数 × --adaptive-ratio)")
    ap.add_argument("--adaptive-ratio", type=float, default=0.45,
                    help="相对阈值系数，按每段视频自身的清晰度中位数取值")
    ap.add_argument("--min-diff", type=float, default=6.0, help="与上一保留帧灰度差低于此值则跳过")
    ap.add_argument("--max-per-video", type=int, default=70)
    ap.add_argument("--only", default="", help="只处理文件名含该子串的视频，如 marker_ / ball_")
    args = ap.parse_args()

    vdir = Path(args.videos)
    if not vdir.is_absolute():
        vdir = ROOT / vdir
    odir = Path(args.out)
    if not odir.is_absolute():
        odir = ROOT / odir

    videos = sorted(p for p in vdir.iterdir() if p.suffix.lower() in VIDEO_EXTS)
    if args.only:
        videos = [p for p in videos if args.only in p.name]
    if not videos:
        print(f"{rel(vdir)} 里没有视频文件")
        return 1

    print("=" * 74)
    print(f"抽帧：{len(videos)} 段视频｜间隔 {args.every}s｜清晰度下限 {args.min_sharp:.0f}"
          f"｜去重差阈值 {args.min_diff:.1f}｜每段上限 {args.max_per_video}")
    print("=" * 74)
    rows = []
    for v in videos:
        stem = v.stem.split("球")[0].split("，")[0].strip() or v.stem
        if not stem.startswith("ball") and not stem.startswith("marker"):
            stem = stem or v.stem
        out = odir / stem
        r = extract_one(v, out, args.every, args.min_sharp, args.min_diff,
                        args.max_per_video, args.adaptive_ratio)
        r["stem"] = stem
        rows.append(r)
        if r.get("err"):
            print(f"{v.name[:44]:<46} {r['err']}")
            continue
        print(f"{v.name[:44]:<46} 采样 {r.get('sampled', 0):3d}  保留 {r['kept']:3d}  "
              f"模糊剔除 {r.get('blur', 0):3d}  重复跳过 {r.get('dup', 0):3d}  "
              f"阈值 {r.get('thr', 0):5.1f}")

    total_kept = sum(r["kept"] for r in rows)
    print("-" * 74)
    print(f"合计保留 {total_kept} 张，输出到 {rel(odir)}")
    if total_kept < 200:
        print(f"警告：不足 200 张（细则要求 ≥200）。可减小 --every（如 1.0）或提高 --max-per-video 重跑。")
    print("下一步：python tools/prelabel.py")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
