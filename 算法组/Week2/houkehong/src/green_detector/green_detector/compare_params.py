#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 2：参数对比实验（每次只改一个主要参数）。

四组实验，选两组以上做：

    hsv   : 只改 HSV 上下界（窄 -> 宽）
    area  : 只改最小轮廓面积（2000 -> 40000）
    morph : 只改是否使用形态学开/闭运算
    light : 参数不变，换成一张光照不同的图

用法：

    # 默认做 hsv 对比，输出 02_参数对比.png
    python3 src/green_detector/green_detector/compare_params.py \
        --experiment hsv --image src/green_detector/test_images/blue_01.png

    # 一次做完：默认做 hsv / area / morph 三组；给了 --image-b 就再加一组光照对比
    # （生成本参数对比_hsv.png / _area.png / _morph.png / _light.png）
    python3 .../compare_params.py --experiment all --image a.jpg --image-b b.jpg

    # 光照对比需要第二张图
    python3 .../compare_params.py --experiment light --image a.jpg --image-b b.jpg
"""

from __future__ import annotations

import argparse
import dataclasses
import os
import sys

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from green_detector.cli_args import add_detector_args, config_from_args
    from green_detector.detector import (annotate, detect, imread_unicode,
                                         imwrite_unicode, mask_to_bgr,
                                         print_banner)
    from green_detector.visual import stack_rows, tile
else:
    from .cli_args import add_detector_args, config_from_args
    from .detector import (annotate, detect, imread_unicode, imwrite_unicode,
                           mask_to_bgr, print_banner)
    from .visual import stack_rows, tile


def build_variants(experiment: str, base):
    """返回 [(标签, cfg), (标签, cfg)]，两个变体之间只差一个参数。"""
    if experiment == "hsv":
        return [
            ("A: H %d-%d S %d-%d" % (base.hsv_lower[0], base.hsv_upper[0],
                                     base.hsv_lower[1], base.hsv_upper[1]),
             base),
            ("B: H 85-140 S 50-255 (更宽)",
             dataclasses.replace(base, hsv_lower=(85, 50, 50), hsv_upper=(140, 255, 255))),
        ]
    if experiment == "area":
        return [
            ("A: min_area=2000", dataclasses.replace(base, min_area=2000.0)),
            ("B: min_area=40000", dataclasses.replace(base, min_area=40000.0)),
        ]
    if experiment == "morph":
        return [
            ("A: 无形态学", dataclasses.replace(base, use_open=False, use_close=False)),
            ("B: 开+闭运算 k=%d" % base.morph_kernel,
             dataclasses.replace(base, use_open=True, use_close=True)),
        ]
    raise ValueError("未知实验: %s" % experiment)


def run_one(img, cfg):
    det = detect(img, cfg)
    return det, annotate(img, det, cfg)


def print_table(title, rows):
    print("\n### %s" % title)
    header = "%-26s %-9s %9s %9s %10s %6s" % ("变体", "detected", "u", "v", "area", "候选")
    print(header)
    print("-" * len(header))
    for label, det in rows:
        if det.detected:
            print("%-26s %-9s %9.1f %9.1f %10.1f %6d"
                  % (label, "True", det.u, det.v, det.area, len(det.candidates)))
        else:
            print("%-26s %-9s %9s %9s %10s %6d"
                  % (label, "False", "-", "-", "-", len(det.candidates)))


def save_comparison(path, img_a, cfg_a, img_b, cfg_b, label_a, label_b, tag):
    det_a, ann_a = run_one(img_a, cfg_a)
    det_b, ann_b = run_one(img_b, cfg_b)
    grid = stack_rows([
        [tile(ann_a, label_a, 520), tile(ann_b, label_b, 520)],
        [tile(mask_to_bgr(det_a.mask), "mask A", 520),
         tile(mask_to_bgr(det_b.mask), "mask B", 520)],
    ])
    if imwrite_unicode(path, grid):
        print("[SAVE] %s 对比图 -> %s" % (tag, os.path.abspath(path)))
    return [(label_a, det_a), (label_b, det_b)]


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="HSV / 面积 / 形态学 / 光照 参数对比")
    p.add_argument("--image", required=True, help="第一张测试图")
    p.add_argument("--image-b", default="",
                   help="light 实验用的第二张图（光照不同）；all 模式下给了就多做一组光照对比")
    p.add_argument("--experiment", default="hsv",
                   choices=["hsv", "area", "morph", "light", "all"],
                   help="all = hsv + area + morph（再加 light 需要 --image-b）")
    p.add_argument("--output", default="02_参数对比.png")
    p.add_argument("--name", default="houkehong")
    add_detector_args(p)
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    base = config_from_args(args)
    print_banner(args.name, "任务2 · 参数对比实验")
    print("[INFO] 基准参数: %s" % base.describe())

    img_a = imread_unicode(args.image)
    if img_a is None:
        print("[ERROR] 读不到图片：%s" % args.image)
        return 2

    if args.experiment == "all":
        todo = ["hsv", "area", "morph"]
        if args.image_b:
            todo.append("light")
    else:
        todo = [args.experiment]

    for exp in todo:
        if exp == "light":
            if not args.image_b:
                print("[ERROR] light 实验需要 --image-b 指定第二张图")
                return 2
            img_b = imread_unicode(args.image_b)
            if img_b is None:
                print("[ERROR] 读不到第二张图：%s" % args.image_b)
                return 2
            suffix = "" if args.experiment != "all" else "_light"
            root, ext = os.path.splitext(args.output)
            out = "%s%s%s" % (root, suffix, ext)
            rows = save_comparison(out, img_a, base, img_b, base,
                                   "A: %s" % os.path.basename(args.image),
                                   "B: %s" % os.path.basename(args.image_b),
                                   "光照")
            print_table("光照对比（参数完全相同）", rows)
            continue

        variants = build_variants(exp, base)
        (label_a, cfg_a), (label_b, cfg_b) = variants
        suffix = "" if args.experiment != "all" else "_" + exp
        root, ext = os.path.splitext(args.output)
        out = "%s%s%s" % (root, suffix, ext)
        rows = save_comparison(out, img_a, cfg_a, img_a, cfg_b, label_a, label_b, exp)

        diff = ""
        if rows[0][1].detected and rows[1][1].detected:
            diff = ("Δu=%.1f, Δv=%.1f, 面积比 %.2f"
                    % (rows[1][1].u - rows[0][1].u,
                       rows[1][1].v - rows[0][1].v,
                       rows[1][1].area / max(rows[0][1].area, 1e-6)))
        elif rows[0][1].detected != rows[1][1].detected:
            diff = "一个变体检出、另一个未检出（漏检/误检的直接证据）"
        print_table("实验 %s" % exp, rows)
        if diff:
            print("[DIFF] %s" % diff)

    print("\n[提示] 把上表的数字和“轮廓是否完整、中心点是否跳动”写进 README 的对比结论。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
