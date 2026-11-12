#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 1 + 任务 2：单张图片的蓝色目标检测与结果可视化。

用法（不需要 ROS2，直接用系统 python3 跑）：

    # 默认输出 02_结果可视化.png
    python3 src/green_detector/green_detector/detect_image.py \
        --image src/green_detector/test_images/blue_01.png

    # 换成宽阈值 + 小面积，看结果怎么变
    python3 .../detect_image.py --image ... --hsv-lower 30 40 40 --hsv-upper 100 255 255 --min-area 200
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# 允许在没装包的情况下直接 python3 本文件运行
if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from green_detector.cli_args import add_detector_args, config_from_args
    from green_detector.detector import (annotate, detect, format_result,
                                         imread_unicode, imwrite_unicode,
                                         mask_to_bgr, print_banner)
else:
    from .cli_args import add_detector_args, config_from_args
    from .detector import (annotate, detect, format_result, imread_unicode,
                           imwrite_unicode, mask_to_bgr, print_banner)


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="HSV 颜色分割 + 轮廓筛选 + 中心点检测")
    p.add_argument("--image", required=True, help="输入图片路径")
    p.add_argument("--output", default="02_结果可视化.png", help="标注结果保存路径")
    p.add_argument("--mask-output", default="", help="mask 保存路径（可选）")
    p.add_argument("--name", default="houkehong", help="终端抬头打印的姓名拼音")
    add_detector_args(p)
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    cfg = config_from_args(args)

    print_banner(args.name, "任务1/任务2 · HSV 颜色分割与中心点检测")

    img = imread_unicode(args.image)
    if img is None:
        print("[ERROR] 读不到图片：%s" % os.path.abspath(args.image))
        print("        检查路径、文件名大小写、扩展名是否是 jpg/png")
        return 2

    h, w = img.shape[:2]
    print("[INFO] 图像: %s" % args.image)
    print("[INFO] 尺寸: %d x %d (宽 x 高), 通道数=%d" % (w, h, img.shape[2]))
    print("[INFO] 参数: %s" % cfg.describe())

    t0 = time.perf_counter()
    det = detect(img, cfg)
    cost_ms = (time.perf_counter() - t0) * 1000.0

    print("[INFO] 通过面积筛选的候选轮廓: %d 个 %s"
          % (len(det.candidates), ["%.0f" % a for a in det.candidates[:5]]))
    if det.detected:
        print("[INFO] 选择规则: 面积最大者胜 -> 面积 %.1f px^2" % det.area)

    # 结果行：成功或失败都只有这一种格式
    if det.detected:
        print("[RESULT] %s" % format_result(det))
        print("[RESULT] 目标中心在图像中的相对位置: u/W=%.3f, v/H=%.3f"
              % (det.u / w, det.v / h))
    else:
        print("[RESULT] %s" % format_result(det))
        print("         提示：先看 mask 图，确认是阈值太严、目标太小，还是目标确实不在图里")
    print("[INFO] 耗时: %.2f ms" % cost_ms)

    annotated = annotate(img, det, cfg)
    if imwrite_unicode(args.output, annotated):
        print("[SAVE] 标注图 -> %s" % os.path.abspath(args.output))
    else:
        print("[ERROR] 标注图保存失败：%s" % args.output)
        return 3

    if args.mask_output and det.mask is not None:
        if imwrite_unicode(args.mask_output, mask_to_bgr(det.mask)):
            print("[SAVE] 二值 mask -> %s" % os.path.abspath(args.mask_output))

    # 返回码：0 表示程序正常跑完（不代表一定检测到目标）
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
