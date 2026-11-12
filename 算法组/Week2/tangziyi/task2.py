#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 2：检测结果可视化 + 参数对比实验

用的还是 OpenCV，检测部分直接复用 task1.py 里的 detect()，不重复写算法。
运行：  python3 task2.py            # 三组对比全跑
        python3 task2.py A          # 只跑 A 组
输出：  output/task2/<图片名>_original_vs_result.png   原图 vs 标注图
        output/task2/param_A_minarea.png              不同 min_area
        output/task2/param_B_hsv.png                  不同 HSV 上界
        output/task2/param_C_morph.png                不同形态学处理
        output/task2/task2_report.md                  参数记录与现象

标注内容：绿色轮廓、橙黄外接矩形、红色中心十字、中心点像素坐标文字。
图像坐标：原点左上角，u 向右、v 向下，单位像素。
"""

import glob
import os
import sys

import cv2
import numpy as np

from task1 import KERNEL_SIZE, MIN_AREA, detect, draw_result, imread_cn, list_images, pad

BASE = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE, "image")
OUT_DIR = os.path.join(BASE, "output", "task2")

FONT = cv2.FONT_HERSHEY_SIMPLEX
CELL_W, CELL_H = 300, 400          # 对比图里每个小格的尺寸
BAR_H = 26

# ============ 三组对比：每次只改一个参数，其余用 task1 的默认值 ============
# (中文说明, 图上英文标签, detect() 的覆盖参数)
EXPERIMENTS = {
    "A": ("A. 最小轮廓面积 min_area", "A. min_area  (300 / 3000 / 60000)", [
        ("min_area=300", "min_area=300", {"min_area": 300}),
        ("min_area=3000 (基准)", "min_area=3000 (base)", {"min_area": 3000}),
        ("min_area=60000", "min_area=60000", {"min_area": 60000}),
    ]),
    "B": ("B. HSV 饱和度上界 S_max（V>=150 不变）", "B. HSV S_max  (40 / 60 / 100)", [
        ("S<=40 (偏紧)", "S<=40 (strict)", {"upper": np.array([180, 40, 255], np.uint8)}),
        ("S<=60 (基准)", "S<=60 (base)", {"upper": np.array([180, 60, 255], np.uint8)}),
        ("S<=100 (偏松)", "S<=100 (loose)", {"upper": np.array([180, 100, 255], np.uint8)}),
    ]),
    "C": ("C. 形态学处理", "C. morphology  (none / open / open+close)", [
        ("不做形态学", "no morph", {"morph": "none"}),
        ("只做开运算", "open only", {"morph": "open"}),
        ("开+闭 (基准)", "open+close (base)", {"morph": "both"}),
    ]),
}


def cell(img, res, caption):
    """把一张图缩成固定大小的小格，顶部加一条标签。"""
    vis = draw_result(img, res)
    vis = cv2.resize(vis, (CELL_W, CELL_H))
    bar = np.full((BAR_H, CELL_W, 3), 45, np.uint8)
    cv2.putText(bar, caption, (5, 18), FONT, 0.5, (255, 255, 255), 1)
    return cv2.vconcat([bar, vis])


def join_row(cells, gap=4):
    """横向拼接，中间留白线，方便区分格子。"""
    parts = []
    for c in cells:
        parts.append(c)
        parts.append(np.full((c.shape[0], gap, 3), 255, np.uint8))
    return cv2.hconcat(parts)


def join_col(rows, gap=4):
    parts = []
    for r in rows:
        parts.append(r)
        parts.append(np.full((gap, r.shape[1], 3), 255, np.uint8))
    return cv2.vconcat(parts)


def title_bar(text, width):
    """对比图最上面的一条标题（只能用英文，cv2 不支持中文字形）。"""
    bar = np.full((36, width, 3), 255, np.uint8)
    cv2.putText(bar, text, (8, 25), FONT, 0.62, (30, 30, 30), 1)
    return bar


def phenomenon(res, ref):
    """给每个结果贴一个简短判断，方便写分析。"""
    if not res["detected"]:
        return "漏检：没有面积达标的轮廓"
    h, w = res["mask"].shape[:2]
    if res["area"] > 0.3 * w * h:
        return "疑似误检：面积占整幅图 %.0f%%" % (100.0 * res["area"] / (w * h))
    if ref and ref["detected"]:
        d = ((res["u"] - ref["u"]) ** 2 + (res["v"] - ref["v"]) ** 2) ** 0.5
        if d > 8:
            return "中心偏移 %.1f px" % d
    return "正常"


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    files = list_images()
    if not files:
        print("image 目录里没有图片，请把 1.png / 2.png / 3.png 放进去。")
        return

    images = []
    for p in files:
        img = imread_cn(p)
        if img is not None:
            images.append((os.path.basename(p), img))

    print("=" * 78)
    print("任务 2：结果可视化与参数对比")
    print("=" * 78)

    # ---------------- 1. 原图 vs 标注图 ----------------
    print("\n[1] 原图 vs 标注图")
    for name, img in images:
        res = detect(img)
        vis = draw_result(img, res)
        h = 480
        a = cv2.resize(img, (int(img.shape[1] * h / img.shape[0]), h))
        b = cv2.resize(vis, (int(vis.shape[1] * h / vis.shape[0]), h))
        out = cv2.hconcat([a, b])
        path = os.path.join(OUT_DIR, name.replace(".png", "_original_vs_result.png"))
        cv2.imwrite(path, out)
        print("    %-8s -> %s" % (name, os.path.basename(path)))

    # ---------------- 2. 参数对比 ----------------
    which = sys.argv[1].upper() if len(sys.argv) > 1 else "ABC"
    report = ["# 任务 2 参数对比记录", "",
              "- 基准参数：min_area=%d，HSV [0,0,150]~[180,60,255]，开+闭运算 椭圆核 %dx%d"
              % (MIN_AREA, KERNEL_SIZE, KERNEL_SIZE),
              "- 控制变量：每次只改一个参数，其它保持基准值。", ""]

    for key, (title, title_en, variants) in EXPERIMENTS.items():
        if key not in which:
            continue
        print("\n" + "-" * 78)
        print("%s" % title)
        print("-" * 78)
        print("%s%s%s%s%s%s" % (pad("参数", 22), pad("图片", 10), pad("结果", 8),
                                pad("u", 9), pad("v", 9), pad("area", 11)) + "候选  Δ中心   现象")

        rows, table = [], []
        refs = {n: detect(img) for n, img in images}     # 基准结果，用来算偏移量
        for label_cn, label_en, kw in variants:
            cells = []
            for name, img in images:
                res = detect(img, **kw)
                cells.append(cell(img, res, "%s | %s" % (label_en, name)))
                ref = refs[name]
                delta = "-"
                if res["detected"] and ref["detected"]:
                    delta = "%.1f" % (((res["u"] - ref["u"]) ** 2 +
                                       (res["v"] - ref["v"]) ** 2) ** 0.5)
                print("%s%s%s%s%s%s%4d %7s   %s"
                      % (pad(label_cn, 22), pad(name, 10),
                         pad("成功" if res["detected"] else "未识别", 8),
                         pad("%.1f" % res["u"], 9), pad("%.1f" % res["v"], 9),
                         pad("%.0f" % res["area"], 11),
                         len(res["candidates"]), delta, phenomenon(res, ref)))
                table.append("| %s | `%s` | %s | %.1f | %.1f | %.0f | %d | %s | %s |"
                             % (label_cn, name, res["detected"], res["u"], res["v"],
                                res["area"], len(res["candidates"]), delta,
                                phenomenon(res, ref)))
            rows.append(join_row(cells))

        grid = cv2.vconcat([title_bar(title_en, rows[0].shape[1]), join_col(rows)])
        path = os.path.join(OUT_DIR, "param_%s.png" % {"A": "A_minarea", "B": "B_hsv",
                                                        "C": "C_morph"}[key])
        cv2.imwrite(path, grid)
        print("   对比图 -> %s" % os.path.basename(path))

        report += ["## " + title, "",
                   "| 参数 | 图片 | detected | u (px) | v (px) | area (px²) | 候选数 | Δ中心 (px) | 现象 |",
                   "| --- | --- | :---: | ---: | ---: | ---: | ---: | ---: | --- |"]
        report += table + ["", "![%s](%s)" % (title, os.path.basename(path)), ""]

    with open(os.path.join(OUT_DIR, "task2_report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(report))

    print("\n" + "=" * 78)
    print("说明：Δ中心 是与基准参数下的中心点距离，用来判断定位是否稳定。")
    print("对比图与表格已保存到：%s" % OUT_DIR)
    print("=" * 78)


if __name__ == "__main__":
    main()
