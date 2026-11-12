# -*- coding: utf-8 -*-
"""任务三：颜色目标提取与轮廓定位

对应任务书要求：
  1. BGR -> HSV，用 inRange 生成二值掩膜
  2. 开/闭运算改善掩膜，findContours 找轮廓，按面积过滤噪声
  3. 在原图上画外接矩形和中心点，输出中心坐标与轮廓面积，保存 mask / result
  4. 至少 3 张不同背景或光照的图片测试，记录稳定 / 误检 / 漏检

运行方式（在项目根目录下）：
    python src/task3_color_detect.py

脚本跑两轮：
  第一轮  三张图统一用"按正常光照调出来的"默认参数
          -> 会看到：正常图 OK、暗光图整个漏掉、水果拼盘多检出香蕉
  第二轮  逐图精调参数 -> 三张图各检出 1 个目标
两轮结果都写进 outputs/task3/report.txt，作为"参数设置依据"的证据。
"""
import os
import sys

import cv2
import numpy as np

from cv_io import imread_u, imwrite_u

OUT_DIR = "outputs/task3"

# 橙色在 OpenCV 里 H 大约是 5~25（OpenCV 的 H 是 0~179，不是 0~360）
# 这个默认范围是按 orange_normal.jpg（光照正常）量出来的
DEFAULT = dict(lower=(5, 120, 120), upper=(25, 255, 255), min_area=800)

# 逐图精调后的参数，每个值为什么这么设写在注释里
CASES = [
    dict(
        path="data/images/orange_normal.jpg",
        cond="橙子特写 / 绿色背景 / 光照正常",
        lower=(5, 120, 120), upper=(25, 255, 255), min_area=3000,
        note="默认范围就够；面积阈值从 800 提到 3000，去掉底部左侧那条 597 像素的红色边缘",
    ),
    dict(
        path="data/images/orange_lowlight.jpg",
        cond="同一张图压暗到 45% / 模拟弱光手持",
        # 暗光下目标 V 从 232 掉到 106，默认的 V>=120 会把整只橙子切没
        # 同时暗部噪点会把 S 拉低，所以 S 下限也从 120 降到 90
        lower=(5, 90, 40), upper=(25, 255, 255), min_area=3000,
        note="V 下限 120->40、S 下限 120->90 才检得出来",
    ),
    dict(
        path="data/images/orange_cluttered.jpg",
        cond="水果拼盘 / 橙子+香蕉+青柠+柠檬+猕猴桃 / 多种黄色系",
        # 右下角的柠檬是黄的，H≈23 落在橙色区间内，S≈133 又过了默认的 S>=120，
        # 于是被当成目标检了出来。先把 S 下限抬到 150，把柠檬主体排掉。
        # 但只调 S 还不够：柠檬最亮的高光处 S 能到 221，还剩一个 2779 像素的小块，
        # 光靠饱和度分不开，所以再加一道面积阈值。
        # 8000 是看两者之间的空隙定的：橙子 8.7 万，柠檬高光 2779，取中间偏上留余量。
        lower=(5, 150, 90), upper=(25, 255, 255), min_area=8000,
        note="S 下限 120->150 排掉柠檬，面积阈值 800->8000 排掉柠檬高光残块",
    ),
]

# 调参时我用的土办法：先取几个点看像素的 HSV 大概落在哪，再往两边放宽。
# 下面这段是量柠檬和橙子用的，留着方便下次换图时直接用（把 # 去掉就能跑）。
# import cv2 as _cv2
# _img = _cv2.imread("data/images/orange_cluttered.jpg")
# _hsv = _cv2.cvtColor(_img, _cv2.COLOR_BGR2HSV)
# print("橙子处 HSV:", _hsv[230, 150])   # (17, 218, 174)  S 很高
# print("柠檬处 HSV:", _hsv[300, 430])   # (24, 132, 147)  H 几乎一样，S 低一截


def detect(path, lower, upper, min_area, tag=None, save=False):
    """在单张图上做颜色提取 + 轮廓定位，返回结果字典。"""
    img = imread_u(path)
    if img is None:
        return dict(ok=False, reason="读不到图片:" + path)

    # ---- 1. BGR -> HSV ----
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # ---- 2. inRange 生成二值掩膜 ----
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))

    # ---- 3. 形态学清理 ----
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)    # 开：去掉小白点噪声
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)   # 闭：补上目标内部的小洞

    # ---- 4. 找轮廓 ----
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = img.copy()
    found = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:              # 面积太小，当噪声丢掉
            continue
        x, y, w, h = cv2.boundingRect(c)
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

        # 画外接矩形（绿）和中心点（红）
        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(result, "(%d,%d)" % (cx, cy), (x, max(y - 8, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 统计这块区域的 HSV 中位数，作为参数依据
        inner = hsv[max(y + h // 4, 0):y + h * 3 // 4,
                    max(x + w // 4, 0):x + w * 3 // 4].reshape(-1, 3)
        med = np.median(inner, axis=0).astype(int) if inner.size else (0, 0, 0)

        found.append(dict(area=area, bbox=(x, y, w, h), center=(cx, cy),
                          hsv=(int(med[0]), int(med[1]), int(med[2]))))

    if save and tag:
        os.makedirs(OUT_DIR, exist_ok=True)
        imwrite_u(os.path.join(OUT_DIR, "mask_%s.png" % tag), mask)
        imwrite_u(os.path.join(OUT_DIR, "result_%s.png" % tag), result)

    return dict(ok=True, n=len(found), found=found, n_candidates=len(contours))


def fmt(res):
    if not res.get("ok"):
        return res.get("reason", "失败")
    if res["n"] == 0:
        return "检出 0 个（掩膜里有 %d 个候选轮廓，全被面积阈值滤掉）" % res["n_candidates"]
    parts = []
    for f in res["found"]:
        parts.append("中心=(%d,%d) 面积=%.0f 外框=%s HSV=%s"
                     % (f["center"][0], f["center"][1], f["area"], f["bbox"], f["hsv"]))
    return "检出 %d 个 | %s" % (res["n"], " ; ".join(parts))


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.makedirs(OUT_DIR, exist_ok=True)

    print("=" * 76)
    print("第一轮：三张图统一用默认参数 lower=%s upper=%s min_area=%d"
          % (DEFAULT["lower"], DEFAULT["upper"], DEFAULT["min_area"]))
    print("=" * 76)
    round1 = {}
    for case in CASES:
        tag = os.path.splitext(os.path.basename(case["path"]))[0]
        r = detect(case["path"], DEFAULT["lower"], DEFAULT["upper"], DEFAULT["min_area"])
        round1[tag] = r
        print("  %-22s %s" % (tag, fmt(r)))

    print()
    print("=" * 76)
    print("第二轮：逐图精调参数")
    print("=" * 76)
    round2 = {}
    for case in CASES:
        tag = os.path.splitext(os.path.basename(case["path"]))[0]
        print("  [%s] %s" % (tag, case["cond"]))
        print("    lower=%s upper=%s min_area=%d"
              % (case["lower"], case["upper"], case["min_area"]))
        r = detect(case["path"], case["lower"], case["upper"],
                   case["min_area"], tag=tag, save=True)
        round2[tag] = r
        print("    -> %s" % fmt(r))
        print("    说明：%s" % case["note"])

    # ---- 写报告 ----
    lines = ["任务三 结果记录", "=" * 64, ""]
    lines.append("第一轮（三张图统一用默认参数）")
    lines.append("-" * 64)
    for case in CASES:
        tag = os.path.splitext(os.path.basename(case["path"]))[0]
        lines.append("%-22s %s" % (tag, fmt(round1[tag])))
    lines.append("")
    lines.append("第二轮（逐图精调）")
    lines.append("-" * 64)
    for case in CASES:
        tag = os.path.splitext(os.path.basename(case["path"]))[0]
        r = round2[tag]
        lines.append("%-22s %s" % (tag, case["cond"]))
        lines.append("    参数: lower=%s upper=%s min_area=%d"
                     % (case["lower"], case["upper"], case["min_area"]))
        lines.append("    结果: %s" % fmt(r))
        lines.append("    依据: %s" % case["note"])
    lines.append("")
    lines.append("参数依据（目标区 HSV 中位数，取自检出框的中心部分）")
    lines.append("-" * 64)
    for case in CASES:
        tag = os.path.splitext(os.path.basename(case["path"]))[0]
        r = round2[tag]
        if r.get("ok") and r["n"]:
            lines.append("%-22s HSV=%s" % (tag, r["found"][0]["hsv"]))

    with open(os.path.join(OUT_DIR, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print()
    print("-" * 76)
    print("产出文件:")
    for name in sorted(os.listdir(OUT_DIR)):
        print("   ", os.path.join(OUT_DIR, name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
