# -*- coding: utf-8 -*-
"""
任务 1：HSV 颜色分割与目标中心点检测（40 分）

链路：读取 BGR 图像 → 转 HSV → 按颜色区间逐一 inRange 并取并集（多段区间用于
     跨 H=0/180 的红色系）→ 形态学去噪 → findContours 提外轮廓 → 面积过滤
     → 轮廓矩计算中心点 (u, v)
"""
import argparse
import csv
import os
import sys

import cv2
import numpy as np

from io_utils import imread, imwrite

# 路径按脚本自身位置解析，不写死任何绝对路径，也不依赖当前工作目录：
#   src/task1.py → 上一级就是项目根目录 → data/images、outputs
# 这样无论从哪个目录运行（IDE 直接 Run、命令行、双击）都能找到同一份数据。
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
IMG_DIR = os.path.join(ROOT_DIR, "data", "images")
OUT_DIR = os.path.join(ROOT_DIR, "outputs")
MAX_SIDE = 800

# 输出里显示的相对路径以这个仓库目录名作第一层
REPO_NAME = "cquick-recruitment-main"


def rel_path(path):
    #打印用的相对路径：以仓库根目录名 cquick-recruitment-main 为第一层。
    target = os.path.abspath(path)
    d = ROOT_DIR
    while True:                                  # 从项目目录向上找仓库根目录
        if os.path.basename(d) == REPO_NAME:
            try:
                rel = os.path.relpath(target, d)
            except ValueError:                   # 跨盘符时无法求相对路径
                return str(path)
            return os.path.join(REPO_NAME, rel)
        parent = os.path.dirname(d)
        if parent == d:                          # 找不到仓库名时退化为项目内相对路径
            try:
                return os.path.relpath(target, ROOT_DIR)
            except ValueError:
                return str(path)
        d = parent

# 每张测试图的 HSV 区间与最小面积（阈值均按该图实测标定）。
# ranges 里每一段都是 (lower, upper)；多段时逐段 inRange 再取并集：
#   - 粉色的色相绕过 H=0/180 两端，单段只能圈住一部分杯身，所以拆成 [0,8] 与 [170,179] 两段；
#   - 黄色胶带与米色桌角色相接近（H≈15），靠饱和度区分（胶带 S≥120，桌角 S≈103）；
#   - 黑色圆柱没有“色相”可言，靠亮度 V 上限切分：目标 V≈35~75，浅色地面 V≈200+，木门 V≈123+。
CONFIG = {
    "obj1.jpg": {"ranges": [((0, 90, 110), (8, 255, 255)),
                            ((170, 90, 110), (179, 255, 255))],
                 "min_area": 3000,
                 "desc": "粉色奶茶杯、室内自然光（手机实拍）"},
    "obj2.jpg": {"ranges": [((10, 120, 90), (25, 255, 255))],
                 "min_area": 3000,
                 "desc": "黄色胶带卷、目标较小（手机实拍）"},
    "obj3.jpg": {"ranges": [((0, 0, 0), (179, 255, 100))],
                 "min_area": 3000,
                 "desc": "黑色圆柱、浅色地面（手机实拍）"},
}


def load_image(path):
    """读取图片并检查是否成功；长边超过 MAX_SIDE 时等比缩小。

    缩小是为了让任务 1、任务 2、任务 3（ROS2 节点）看到同一套像素尺寸，
    保证三处输出的 (u, v) 可以互相对照。读不到图返回 None。
    """
    img = imread(path)
    if img is None:
        return None
    h, w = img.shape[:2]
    scale = MAX_SIDE / max(h, w)
    if scale < 1:
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return img


def detect(img, ranges, min_area, use_morphology=True):
    #颜色目标检测核心函数。

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 多段颜色区间：逐段 inRange，再按位或合并成一张 mask
    mask = None
    for lower, upper in ranges:
        m = cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = m if mask is None else cv2.bitwise_or(mask, m)

    # 可选形态学：开运算去小白点 + 闭运算补小黑洞
    if use_morphology:
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    #提取轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 面积过滤：只保留面积 >= min_area 的轮廓
    candidates = []
    for c in contours:
        a = cv2.contourArea(c)
        if a >= min_area:
            candidates.append((a, c))

    # 没有候选 → 明确的“未检测到”，坐标用约定默认值 0
    if not candidates:
        return False, 0.0, 0.0, 0.0, mask, candidates

    # 多个候选 → 按面积降序，取最大者
    candidates.sort(key=lambda t: -t[0])
    area, best = candidates[0]

    # 中心点用轮廓矩计算，处理 m00 == 0 的除零情况
    M = cv2.moments(best)
    if M["m00"] == 0:
        return False, 0.0, 0.0, 0.0, mask, candidates

    u = float(M["m10"] / M["m00"])
    v = float(M["m01"] / M["m00"])
    return True, u, v, float(area), mask, candidates


def run():
    #对 CONFIG 里的 3 张测试图跑检测，保存 mask 与 CSV。
    rows = []
    print("=== 任务1：颜色目标检测（3 张测试图） ===\n")
    for name, cfg in CONFIG.items():
        path = os.path.join(IMG_DIR, name)
        img = load_image(path)
        if img is None:
            print(f"[错误] 读取失败: {rel_path(path)}")
            continue

        detected, u, v, area, mask, cands = detect(
            img, cfg["ranges"], cfg["min_area"])

        if detected:
            print(f"[{name}] 检测到目标: u={u:.1f}, v={v:.1f}, area={area:.0f} "
                  f"(候选数={len(cands)})  [{cfg['desc']}]")
        else:
            print(f"[{name}] 未检测到目标 (detected=false, u=v=area=0)  [{cfg['desc']}]")

        stem = os.path.splitext(name)[0]
        imwrite(os.path.join(OUT_DIR, f"{stem}_mask.png"), mask)
        rows.append([name, detected, round(u, 1), round(v, 1), round(area, 0), len(cands)])

    with open(os.path.join(OUT_DIR, "detect_result.csv"), "w",
              newline="", encoding="utf-8-sig") as f:
        wtr = csv.writer(f)
        wtr.writerow(["image", "detected", "u", "v", "area", "num_candidates"])
        wtr.writerows(rows)

    print("\n结果已保存到:", rel_path(OUT_DIR))


def demo_no_target():
    """演示失败分支：用不可能命中的阈值跑 obj1，证明“没检测到”时不会返回未初始化坐标。"""
    path = os.path.join(IMG_DIR, "obj1.jpg")
    img = load_image(path)
    if img is None:
        print(f"[错误] 读取失败: {rel_path(path)}")
        return
    detected, u, v, area, mask, cands = detect(
        img, [((170, 255, 255), (179, 255, 255))], 3000)  # 全饱和紫色，图里不存在
    print("\n=== 失败分支演示（obj1 + 不存在的颜色范围） ===")
    if not detected and u == v == area == 0.0:
        print("未检测到目标：detected=false, u=v=area=0（坐标有明确默认值，"
              "不会被误当成图像左上角）")
    else:
        print(f"[异常] 未按预期失败: detected={detected} u={u} v={v} area={area}")


def main():
    parser = argparse.ArgumentParser(description="任务1：HSV 颜色分割与目标中心点检测")
    parser.add_argument("--demo-no-target", action="store_true",
                        help="额外演示未检测到目标时的失败分支")
    args = parser.parse_args()

    if not os.path.isdir(IMG_DIR):
        sys.exit(f"[错误] 找不到 {rel_path(IMG_DIR)}（数据目录应与 src/ 同级）")
    os.makedirs(OUT_DIR, exist_ok=True)
    run()
    if args.demo_no_target:
        demo_no_target()


if __name__ == "__main__":
    main()
