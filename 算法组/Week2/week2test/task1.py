#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 1：HSV 颜色分割 → 形态学处理 → 外轮廓 → 面积筛选 → 目标中心点 (u, v)

只用到 OpenCV(cv2) + numpy，没有别的依赖。
运行：  python3 task1.py
输出：  终端打印每张图的候选目标与最终选择
        output/task1/<图片名>_mask.png    二值掩膜
        output/task1/<图片名>_result.png  检测结果（轮廓 + 中心点 + 坐标）
        output/task1/task1_report.md      结果表格

图像坐标约定：原点在左上角，u 向右增大，v 向下增大，单位是像素。
"""

import glob
import os
import unicodedata

import cv2
import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE, "image")
OUT_DIR = os.path.join(BASE, "output", "task1")

# ============ 可调参数（README 记录的就是这 5 条） ============
TARGET_COLOR = "白色足球（低饱和度 + 高亮度）"
HSV_LOWER = np.array([0, 0, 150], np.uint8)      # H, S, V 下界
HSV_UPPER = np.array([180, 60, 255], np.uint8)   # H, S, V 上界
MIN_AREA = 3000                                   # 最小轮廓面积（像素平方）
KERNEL_SIZE = 7                                   # 形态学核大小（椭圆核）
# =============================================================


def imread_cn(path):
    """读取图片。cv2.imread 遇到中文路径会返回 None，这里用 imdecode 兜底。"""
    data = np.fromfile(path, dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def list_images(image_dir=IMAGE_DIR):
    """列出目录下的图片（png/jpg/bmp），按文件名排序。"""
    files = []
    for ext in ("*.png", "*.jpg", "*.jpeg", "*.bmp"):
        files += glob.glob(os.path.join(image_dir, ext))
    return sorted(files)


def detect(img, lower=HSV_LOWER, upper=HSV_UPPER, min_area=MIN_AREA,
           kernel_size=KERNEL_SIZE, morph="both"):
    """核心检测函数。

    morph 可选："both"（开+闭，默认）/"open"/"close"/"none"，方便做参数对比。

    返回字典：
        detected    bool    是否找到目标
        u, v        float   中心点像素坐标（未找到时为 0）
        area        float   目标轮廓面积（未找到时为 0）
        candidates  list    面积达标的候选，已按面积降序排列
        contour             最终选中的轮廓
        box                 (x, y, w, h) 外接矩形
        mask                二值掩膜
    """
    # 1) BGR -> HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 2) 根据 HSV 范围生成二值掩膜（目标=255，背景=0）
    mask = cv2.inRange(hsv, lower, upper)

    # 3) 形态学处理：开运算去噪点，闭运算补上球面黑斑造成的小洞
    if morph != "none":
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        if morph in ("open", "both"):
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        if morph in ("close", "both"):
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    # 4) 提取外轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 5) 按面积筛选，并按面积从大到小排序
    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area >= min_area:
            candidates.append({"area": area, "contour": c, "box": cv2.boundingRect(c)})
    candidates.sort(key=lambda d: d["area"], reverse=True)

    result = {"detected": False, "u": 0.0, "v": 0.0, "area": 0.0,
              "candidates": candidates, "contour": None, "box": None, "mask": mask}
    if not candidates:
        return result                       # 没找到：u/v/area 全部保持 0，不返回假坐标

    # 6) 多个候选时选面积最大的那个
    best = candidates[0]

    # 7) 用轮廓矩算中心点；m00 为 0（轮廓退化成点/线）时退回外接矩形中心
    u, v = center_of(best["contour"], best["box"])

    result.update(detected=True, u=u, v=v, area=best["area"],
                  contour=best["contour"], box=best["box"])
    return result


def center_of(contour, box):
    """给候选列表用的小函数：算某个候选的中心点。"""
    m = cv2.moments(contour)
    if m["m00"] > 0:
        return m["m10"] / m["m00"], m["m01"] / m["m00"]
    x, y, w, h = box
    return x + w / 2.0, y + h / 2.0


def pad(text, width):
    """按显示宽度补空格（中文占两列），让表格能对齐。"""
    text = str(text)
    w = sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in text)
    return text + " " * max(0, width - w)


def draw_result(img, res):
    """把轮廓、中心点和坐标画回原图。"""
    vis = img.copy()
    if not res["detected"]:
        cv2.putText(vis, "NO TARGET", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        return vis

    cv2.drawContours(vis, [res["contour"]], -1, (0, 255, 0), 3)          # 绿色轮廓
    x, y, w, h = res["box"]
    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 200, 255), 2)          # 橙黄外接矩形
    cu, cv_ = int(res["u"]), int(res["v"])
    cv2.drawMarker(vis, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 26, 3)  # 红色十字

    text = "u=%d, v=%d" % (cu, cv_)
    ty = y - 14 if y > 40 else y + h + 34          # 默认写在框上方，挤不下就写下方
    cv2.putText(vis, text, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 5)   # 黑色描边
    cv2.putText(vis, text, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)  # 红色文字
    return vis


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    files = list_images()

    print("=" * 74)
    print("任务 1：HSV 颜色分割与目标中心点检测")
    print("=" * 74)
    print("目标颜色 : %s" % TARGET_COLOR)
    print("HSV 阈值 : [%d, %d, %d] ~ [%d, %d, %d]"
          % (*HSV_LOWER, *HSV_UPPER))
    print("最小面积 : %d px^2" % MIN_AREA)
    print("形态学   : 开运算 + 闭运算，椭圆核 %dx%d" % (KERNEL_SIZE, KERNEL_SIZE))
    print("图片目录 : %s" % IMAGE_DIR)
    print("-" * 74)

    if not files:
        print("image 目录里没有图片，请把 1.png / 2.png / 3.png 放进去。")
        return

    rows = []
    ok = 0
    for i, path in enumerate(files, 1):
        name = os.path.basename(path)
        img = imread_cn(path)
        if img is None:
            print("[%d/%d] %s  读取失败" % (i, len(files), name))
            continue

        h, w = img.shape[:2]
        res = detect(img)
        print("[%d/%d] %s  尺寸 %dx%d" % (i, len(files), name, w, h))

        # —— 多个候选：按面积排序后逐个打印 ——
        cands = res["candidates"]
        print("      候选目标 %d 个（按轮廓面积降序）：" % len(cands))
        for j, c in enumerate(cands, 1):
            cu, cv_ = center_of(c["contour"], c["box"])
            x, y, cw, ch = c["box"]
            print("         %d) area=%8.0f px^2  bbox=(x=%d, y=%d, w=%d, h=%d)  中心=(%.1f, %.1f)"
                  % (j, c["area"], x, y, cw, ch, cu, cv_))

        if res["detected"]:
            ok += 1
            print("      >>> 最终选择第 1 个（面积最大）：u=%.1f, v=%.1f, area=%.0f"
                  % (res["u"], res["v"], res["area"]))
        else:
            print("      >>> 未识别到目标（detected=False，u=v=area=0）")

        # 保存掩膜和结果图
        cv2.imwrite(os.path.join(OUT_DIR, name.replace(".png", "_mask.png")), res["mask"])
        cv2.imwrite(os.path.join(OUT_DIR, name.replace(".png", "_result.png")),
                    draw_result(img, res))

        rows.append((name, w, h, res))
        print("")

    # —— 汇总 ——
    print("=" * 74)
    print("汇总：%d 张图，检测成功 %d 张，未识别 %d 张"
          % (len(files), ok, len(files) - ok))
    print("%s%s%s%s%s%s" % (pad("文件", 10), pad("尺寸(WxH)", 14), pad("结果", 10),
                            pad("u", 10), pad("v", 10), "area(px^2)"))
    for name, w, h, res in rows:
        print("%s%s%s%s%s%.0f"
              % (pad(name, 10), pad("%dx%d" % (w, h), 14),
                 pad("成功" if res["detected"] else "未识别", 10),
                 pad("%.1f" % res["u"], 10), pad("%.1f" % res["v"], 10), res["area"]))
    print("=" * 74)

    # —— 写一份 md 表格 ——
    lines = ["# 任务 1 结果", "",
             "- 目标颜色：%s" % TARGET_COLOR,
             "- HSV 阈值：[%d, %d, %d] ~ [%d, %d, %d]" % (*HSV_LOWER, *HSV_UPPER),
             "- 最小轮廓面积：%d px²" % MIN_AREA,
             "- 中心点方法：轮廓矩 m10/m00, m01/m00；m00=0 时用外接矩形中心",
             "",
             "| 图片 | 尺寸(WxH) | detected | u (px) | v (px) | area (px²) | 候选数 |",
             "| --- | --- | :---: | ---: | ---: | ---: | ---: |"]
    for name, w, h, res in rows:
        lines.append("| `%s` | %dx%d | %s | %.1f | %.1f | %.0f | %d |"
                     % (name, w, h, res["detected"], res["u"], res["v"],
                        res["area"], len(res["candidates"])))
    lines += ["", "> u/v 是图像像素坐标（原点左上角，u 向右、v 向下），不是机器人坐标或米制距离。", ""]
    with open(os.path.join(OUT_DIR, "task1_report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("结果已保存到：%s" % OUT_DIR)


if __name__ == "__main__":
    main()
