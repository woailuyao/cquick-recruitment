#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把你桌面上「图」文件夹里的照片整理成规范的测试集。

做的事：
  1. 读源目录里所有图片（名字乱、扩展名不对都没关系，按内容解码）；
  2. 用默认蓝色阈值各跑一遍检测；
  3. 按检出面积从大到小重命名成 blue_01.png、blue_02.png ……（没检出的排最后，
     那张天然就是「无目标」测试图）；
  4. 再把面积最大的那张压暗到 40% 存成 blue_0N.png —— 用来演示「漏检」。

用法（在 houkehong/ 目录下）：

    python3 tools/prepare_test_images.py
    python3 tools/prepare_test_images.py --src "/mnt/c/Users/Administrator/Desktop/图"

注意：源目录里原有的 green_0*.jpg（之前生成的绿色合成图）不会被自动删除，
      请先手动 `rm src/green_detector/test_images/green_0*.jpg`，否则批量测试会把它们一起跑。
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src", "green_detector"))

from green_detector.detector import (DetectorConfig, detect, imread_unicode,  # noqa: E402
                                     imwrite_unicode)

EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def main() -> int:
    ap = argparse.ArgumentParser(description="把桌面照片整理成 blue_01..N 测试集")
    ap.add_argument("--src", default="/mnt/c/Users/Administrator/Desktop/图",
                    help="源图片目录，默认桌面上的「图」文件夹")
    ap.add_argument("--out", default="src/green_detector/test_images",
                    help="输出目录，默认包的 test_images")
    ap.add_argument("--dark-factor", type=float, default=0.25,
                    help="暗光图的亮度系数，默认 0.25（V 会掉到 60 以下 -> 漏检）")
    args = ap.parse_args()

    if not os.path.isdir(args.src):
        print("[ERROR] 源目录不存在: %s" % args.src)
        return 2
    names = sorted(n for n in os.listdir(args.src) if n.lower().endswith(EXTS))
    if not names:
        print("[ERROR] 源目录里没有图片: %s" % args.src)
        return 2

    cfg = DetectorConfig()
    print("[INFO] 参数: %s" % cfg.describe())
    print("[INFO] 源目录: %s（%d 张）\n" % (args.src, len(names)))

    rows = []
    for name in names:
        path = os.path.join(args.src, name)
        img = imread_unicode(path)
        if img is None:
            print("[WARN] 读不了，跳过: %s" % name)
            continue
        det = detect(img, cfg)
        h, w = img.shape[:2]
        rows.append({"name": name, "img": img, "det": det, "w": w, "h": h})

    # 检出面积大的排前面，没检出的（面积 0）自然落到最后
    rows.sort(key=lambda r: (r["det"].area, r["w"] * r["h"]), reverse=True)

    os.makedirs(args.out, exist_ok=True)
    print("%-22s %-16s %-9s %10s %11s" % ("原文件", "新文件名", "detected", "area", "尺寸"))
    print("-" * 74)

    for i, r in enumerate(rows, start=1):
        new_name = "blue_%02d.png" % i
        out_path = os.path.join(args.out, new_name)
        if not imwrite_unicode(out_path, r["img"]):
            print("[ERROR] 写文件失败: %s" % out_path)
            return 3
        r["new"] = new_name
        print("%-22s %-16s %-9s %10.1f %11s"
              % (r["name"][:20], new_name,
                 "True" if r["det"].detected else "False",
                 r["det"].area, "%dx%d" % (r["w"], r["h"])))

    # 暗光图：拿面积最大的那张，亮度压下去
    big = rows[0]
    dark = np.clip(big["img"].astype(np.float32) * args.dark_factor, 0, 255).astype(np.uint8)
    dark_name = "blue_%02d.png" % (len(rows) + 1)
    dark_path = os.path.join(args.out, dark_name)
    imwrite_unicode(dark_path, dark)
    det_dark = detect(dark, cfg)
    print("%-22s %-16s %-9s %10.1f %11s"
          % ("(由 %s 压暗生成)" % big["new"], dark_name,
             "True" if det_dark.detected else "False", det_dark.area,
             "%dx%d" % (big["w"], big["h"])))

    print("\n[INFO] 共写出 %d 张到 %s/" % (len(rows) + 1, args.out))
    print("[NEXT] 跑批量测试看汇总：")
    print("       python3 src/green_detector/green_detector/batch_test.py \\")
    print("           --image-dir %s --output 02_批量结果.png" % args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
