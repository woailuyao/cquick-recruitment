#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 1 补充：批量跑若干张测试图片，打印汇总表并拼一张结果图。

加分项「对至少 5 张图片做测试，并总结误检/漏检」就是靠这个脚本出数据。

用法：

    python3 src/green_detector/green_detector/batch_test.py \
        --image-dir src/green_detector/test_images \
        --output 02_批量结果.png --out-dir out
"""

from __future__ import annotations

import argparse
import os
import sys
import time

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

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")


def list_images(folder: str):
    if not os.path.isdir(folder):
        return []
    names = sorted(n for n in os.listdir(folder) if n.lower().endswith(IMAGE_EXTS))
    return [os.path.join(folder, n) for n in names]


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="批量检测测试图片并输出汇总表")
    p.add_argument("--image-dir", default="src/green_detector/test_images")
    p.add_argument("--output", default="02_批量结果.png", help="拼图保存路径")
    p.add_argument("--out-dir", default="out", help="单张标注图与 mask 的保存目录")
    p.add_argument("--name", default="houkehong")
    add_detector_args(p)
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    cfg = config_from_args(args)
    print_banner(args.name, "任务1 · 多图批量测试")

    paths = list_images(args.image_dir)
    if not paths:
        print("[ERROR] 目录里没有图片：%s" % os.path.abspath(args.image_dir))
        print("        支持 %s，把测试图按 blue_01.png 这样命名放进去" % (IMAGE_EXTS,))
        return 2

    print("[INFO] 参数: %s" % cfg.describe())
    print("[INFO] 共 %d 张图片\n" % len(paths))

    header = "%-3s %-18s %-9s %9s %9s %11s %6s %9s" % (
        "#", "文件", "detected", "u", "v", "area", "候选", "耗时ms")
    print(header)
    print("-" * len(header))

    ok_count, tiles, rows = 0, [], []
    for i, path in enumerate(paths, start=1):
        img = imread_unicode(path)
        name = os.path.basename(path)
        if img is None:
            print("%-3d %-18s  读图失败" % (i, name))
            continue
        t0 = time.perf_counter()
        det = detect(img, cfg)
        cost = (time.perf_counter() - t0) * 1000.0
        if det.detected:
            ok_count += 1
            print("%-3d %-18s %-9s %9.1f %9.1f %11.1f %6d %9.2f"
                  % (i, name, "True", det.u, det.v, det.area, len(det.candidates), cost))
        else:
            print("%-3d %-18s %-9s %9s %9s %11s %6d %9.2f"
                  % (i, name, "False", "-", "-", "-", len(det.candidates), cost))

        annotated = annotate(img, det, cfg)
        for out_path, image in (
                (os.path.join(args.out_dir, "annot_" + name), annotated),
                (os.path.join(args.out_dir, "mask_" + name), mask_to_bgr(det.mask))):
            if not imwrite_unicode(out_path, image):
                print("[WARN] 保存失败（检查目录权限/磁盘）: %s" % out_path)
        tiles.append(tile(annotated, "%s  %s" % (name, "OK" if det.detected else "NO TARGET"),
                          width=420))

    print("-" * len(header))
    print("[SUMMARY] 共 %d 张，检出 %d 张，未检出 %d 张"
          % (len(paths), ok_count, len(paths) - ok_count))
    print("[INFO] 单张标注图与 mask 已保存到 %s/" % args.out_dir)

    if tiles:
        cols = 2
        for i in range(0, len(tiles), cols):
            rows.append(tiles[i:i + cols])
        grid = stack_rows(rows)
        if imwrite_unicode(args.output, grid):
            print("[SAVE] 汇总拼图 -> %s" % os.path.abspath(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
