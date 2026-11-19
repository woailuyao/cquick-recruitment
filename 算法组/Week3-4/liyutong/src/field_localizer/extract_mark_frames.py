# -*- coding: utf-8 -*-
"""
标志点抽帧：逐帧过质量闸门，只把清晰、不过曝、有白线的帧写出来

和 extract_frames.py 的区别
    extract_frames.py 给足球数据用的，按固定时间间隔无脑抽。
    标志点这批素材质量参差不齐（过曝/发虚），糊帧标出来就是训练噪声，
    所以这里逐帧算指标，不达标的直接跳过；再加上"和上一张差异太小就跳"，
    避免连续几十张几乎一样的图。

用法
    python extract_mark_frames.py                       # 全部段，按类别自动配密度
    python extract_mark_frames.py --all                 # 同上（显式）
    python extract_mark_frames.py --seg L_5 T_2         # 只做指定段
    python extract_mark_frames.py --dry                 # 只统计不写盘

密度按类别分开定
    L 段最多（16 段），每段 8 张就够；T/X 段少（6/5 段），每段多抽一些，
    免得视角多样性不够；neg 只要背景，6 张足够。

输出
    raw_mark/<段>/<段>_001.jpg
"""
import argparse
import glob
import os
import shutil

import cv2
import numpy as np

from screen_mark_videos import score_frame, verdict

VIDEO_DIR = "videos"
OUT_ROOT = "raw_mark"
KEEP_LIST = "mark_keep.txt"
PREFIXES = ("L", "T", "X", "neg")

# 每段最多抽几张 / 抽帧间隔（秒）
CAP = {"L": 8, "T": 14, "X": 14, "neg": 6}
INTERVAL = {"L": 1.2, "T": 0.8, "X": 0.8, "neg": 1.5}


def natural_key(s):
    base = os.path.splitext(os.path.basename(s))[0]
    parts = base.split("_")
    pre = parts[0]
    num = int("".join(c for c in parts[1] if c.isdigit()) or 0) if len(parts) > 1 else 0
    return (pre, num)


def frame_diff(a, b):
    """两帧的灰度平均绝对差，用来做多样性去重"""
    ga = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY).astype(np.float32)
    gb = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY).astype(np.float32)
    return float(np.abs(ga - gb).mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=VIDEO_DIR)
    ap.add_argument("--dst", default=OUT_ROOT)
    ap.add_argument("--interval", type=float, default=None,
                    help="抽帧间隔（秒）。不传则按类别用内置值 L1.2 / T0.8 / X0.8 / neg1.5")
    ap.add_argument("--max-per-video", type=int, default=None,
                    help="每段最多保留几张。不传则按类别用内置值 L8 / T14 / X14 / neg6")
    ap.add_argument("--min-sharp", type=float, default=120.0)
    ap.add_argument("--max-over", type=float, default=26.0, help="过曝像素上限 %")
    ap.add_argument("--min-white", type=float, default=1.5, help="白线占比下限 %")
    ap.add_argument("--min-diff", type=float, default=1.2,
                    help="和上一张保留帧的最小差异（灰度平均绝对差）")
    ap.add_argument("--seg", nargs="*", default=None, help="只做指定段")
    ap.add_argument("--all", action="store_true", help="忽略 mark_keep.txt，全部段都做")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--clean", action="store_true", help="先清空目标目录")
    args = ap.parse_args()

    if args.seg:
        names = list(args.seg)
    elif args.all or not os.path.exists(KEEP_LIST):
        names = [os.path.splitext(os.path.basename(p))[0]
                 for pre in PREFIXES
                 for p in glob.glob(os.path.join(args.src, f"{pre}_*.mp4"))]
        names.sort(key=lambda s: natural_key(s + ".mp4"))
        if not args.all:
            print("没有 mark_keep.txt，按全部段处理（逐帧闸门会筛掉烂帧）")
    else:
        names = [l.strip() for l in open(KEEP_LIST, encoding="utf-8") if l.strip()]
        print(f"读 {KEEP_LIST}：{len(names)} 段（想用全部段请加 --all）")

    if args.clean and not args.dry:
        for n in names:
            d = os.path.join(args.dst, n)
            if os.path.isdir(d):
                shutil.rmtree(d)
        print("已清空目标段目录")

    total = 0
    print(f"{'段':<8} {'抽帧':>5} {'保留':>5} {'达标率':>7}  跳过原因")
    print("-" * 62)
    for n in names:
        vp = os.path.join(args.src, n + ".mp4")
        if not os.path.exists(vp):
            print(f"{n:<8} 缺视频")
            continue
        cap = cv2.VideoCapture(vp)
        if not cap.isOpened():
            print(f"{n:<8} 打不开")
            continue
        pre = n.split("_")[0]
        interval = args.interval if args.interval else INTERVAL.get(pre, 1.0)
        cap_n = args.max_per_video if args.max_per_video else CAP.get(pre, 10)
        fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
        step = max(1, int(round(interval * fps)))
        d = os.path.join(args.dst, n)
        if not args.dry:
            os.makedirs(d, exist_ok=True)

        tried = kept = 0
        reasons = {}
        last = None
        i = 0
        while True:
            ok, fr = cap.read()
            if not ok:
                break
            if i % step == 0:
                tried += 1
                s = score_frame(fr)
                v = verdict(s, args.min_sharp, args.max_over, args.min_white)
                if v != "OK":
                    reasons["质量:" + v] = reasons.get("质量:" + v, 0) + 1
                elif last is not None and frame_diff(fr, last) < args.min_diff:
                    reasons["雷同"] = reasons.get("雷同", 0) + 1
                elif kept >= cap_n:
                    reasons["超配额"] = reasons.get("超配额", 0) + 1
                else:
                    kept += 1
                    last = fr.copy()
                    if not args.dry:
                        cv2.imwrite(os.path.join(d, f"{n}_{kept:03d}.jpg"), fr,
                                    [cv2.IMWRITE_JPEG_QUALITY, 95])
            i += 1
        cap.release()
        total += kept if not args.dry else 0
        rate = (kept / tried * 100) if tried else 0
        rs = "  ".join(f"{k}{v}" for k, v in sorted(reasons.items()))
        print(f"{n:<8} {tried:5d} {kept:5d} {rate:6.0f}%  {rs}")

    print("-" * 62)
    if args.dry:
        print("dry-run，未写盘")
    else:
        print(f"共写出 {total} 张 -> {args.dst}/")
        print("下一步标注（每帧只有 1~3 个标志点，一轮约 25 分钟）：")
        print("  python label_tool.py L_5 --cls auto --raw raw_mark --scale 2.0")


if __name__ == "__main__":
    main()
