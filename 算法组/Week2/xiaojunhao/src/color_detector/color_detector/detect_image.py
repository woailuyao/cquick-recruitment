# -*- coding: utf-8 -*-
"""对一组图片跑颜色检测，打印中心坐标并把标注图存下来。

这个脚本不需要 ROS2，装着 opencv 就能跑，方便先把算法调好。

用法:
  python -m color_detector.detect_image
  python -m color_detector.detect_image --dir data/images --min-area 800
  python -m color_detector.detect_image --s-min 60 --no-morph
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

from . import core, paths

# data/images 相对于本包往上两级（color_detector/ 的父目录）
PKG_ROOT = Path(__file__).resolve().parents[1]


def imread_u(path):
    """读图。文件不存在或读不出来就返回 None。"""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    return None if data.size == 0 else cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_u(path, img):
    ok, buf = cv2.imencode(Path(path).suffix or ".png", img)
    if ok:
        buf.tofile(str(path))
    return bool(ok)


def main():
    ap = argparse.ArgumentParser(description="颜色目标检测（单张图片或整个目录）")
    ap.add_argument("--dir", default=str(paths.image_dir()),
                    help="图片目录，默认 data/images")
    ap.add_argument("--image", help="只处理这一张")
    ap.add_argument("--out", default=str(PKG_ROOT / "results"),
                    help="标注图输出目录，默认 results")
    ap.add_argument("--h-min", type=int, default=core.DEFAULT_LOWER[0])
    ap.add_argument("--h-max", type=int, default=core.DEFAULT_UPPER[0])
    ap.add_argument("--s-min", type=int, default=core.DEFAULT_LOWER[1])
    ap.add_argument("--v-min", type=int, default=core.DEFAULT_LOWER[2])
    ap.add_argument("--min-area", type=int, default=core.DEFAULT_MIN_AREA)
    ap.add_argument("--no-morph", action="store_true", help="不做形态学处理")
    ap.add_argument("--quiet", action="store_true", help="只打印表格，不存图")
    args = ap.parse_args()

    lower = (args.h_min, args.s_min, args.v_min)
    upper = (args.h_max, 255, 255)
    morph = not args.no_morph

    if args.image:
        files = [Path(args.image)]
    else:
        d = Path(args.dir)
        if not d.is_dir():
            print("目录不存在: %s" % d)
            return 2
        files = sorted([p for p in d.iterdir()
                        if p.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp")])
    if not files:
        print("没有找到图片")
        return 2

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print("HSV 下界=%s 上界=%s 最小面积=%d 形态学=%s"
          % (lower, upper, args.min_area, "开+闭" if morph else "关"))
    print("-" * 76)
    print("%-18s %-6s %-14s %-8s %-16s %s"
          % ("图片", "结果", "中心 (u, v)", "面积", "外接矩形", "候选数"))
    print("-" * 76)

    hit = 0
    for p in files:
        img = imread_u(p)
        if img is None:
            print("%-18s 读取失败" % p.name)
            continue

        target, mask, contours = core.detect(img, lower, upper, args.min_area, morph)
        if target is None:
            # 明确报告没找到，不能拿 (0, 0) 冒充
            print("%-18s %-6s %-14s %-8s %-16s %d"
                  % (p.name, "未检测到", "-", "-", "-", len(contours)))
            if not args.quiet:
                imwrite_u(out_dir / ("%s_mask.png" % p.stem), mask)
                imwrite_u(out_dir / ("%s_result.png" % p.stem), core.annotate(img, None))
            continue

        hit += 1
        # 像素坐标原点是左上角，u 向右变大，v 向下变大
        print("%-18s %-6s %-14s %-8d %-16s %d"
              % (p.name, "命中",
                 "(%d, %d)" % (round(target.u), round(target.v)),
                 int(target.area),
                 "(%d,%d,%d,%d)" % target.bbox,
                 target.candidates))
        if not args.quiet:
            imwrite_u(out_dir / ("%s_mask.png" % p.stem), mask)
            imwrite_u(out_dir / ("%s_result.png" % p.stem),
                      core.annotate(img, target))

    print("-" * 76)
    print("共 %d 张，检测到 %d 张，未检测到 %d 张"
          % (len(files), hit, len(files) - hit))
    if not args.quiet:
        print("标注图输出到: %s" % out_dir)
    return 0 if hit else 1


if __name__ == "__main__":
    sys.exit(main())
