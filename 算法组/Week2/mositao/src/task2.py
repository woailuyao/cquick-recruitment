# -*- coding: utf-8 -*-
"""任务 2：检测结果可视化与参数对比（30 分）

运行（任意工作目录均可；脚本按自身位置找 data/images 与 outputs）：
    python src/task2.py

输出：
    outputs/obj*_result.png          原图 + 轮廓/矩形 + 中心点 + 坐标文字
    outputs/cmp1_upper_combined.png  对比1：HSV V 上限（A=100 / B=160，np.hstack 拼接）
    outputs/cmp1_lower_combined.png  对比1：HSV V 下限（A=0 / B=80）
    outputs/cmp2_combined.png        对比2：最小轮廓面积（A=500 / B=3000）
    outputs/cmp3_combined.png        对比3：是否做形态学开闭运算（A=不做 / B=开+闭）
    outputs/cmp4_light_combined.png  对比4：obj2 原光照 vs 暖光版（同一套阈值，A=原图 / B=光照变化）
    终端                             每组的 detected / u / v / area / 白占比 / 候选数
"""
import os
import sys
from datetime import datetime

import cv2
import numpy as np

from io_utils import imwrite
from task1 import CONFIG, IMG_DIR, OUT_DIR, detect, load_image, rel_path

STAMP_NAME = "mositao"


# 在图片右下角写「姓名拼音 + 实时时间戳」，带半透明底条。
def add_stamp(img):
    text = f"{STAMP_NAME}  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
    h, w = img.shape[:2]
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    x, y = w - tw - 14, h - 12
    overlay = img.copy()
    cv2.rectangle(overlay, (x - 8, y - th - 8), (x + tw + 8, y + base + 4), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return img


# 在原图上画出轮廓/外接框 + 中心点 + 坐标文字 + 姓名时间戳。
def annotate(img, detected, u, v, area, candidates):
    vis = img.copy()
    if detected:
        for i, (a, c) in enumerate(candidates[:3]):  # 最多画前 3 个候选
            x, y, w, h = cv2.boundingRect(c)
            color = (0, 255, 0) if i == 0 else (0, 200, 200)
            cv2.drawContours(vis, [c], -1, color, 2)
            cv2.rectangle(vis, (x, y), (x + w, y + h), color, 2)
        ui, vi = int(round(u)), int(round(v))
        cv2.circle(vis, (ui, vi), 6, (0, 0, 255), -1)
        cv2.putText(vis, f"u={ui}, v={vi} A={int(area)}",
                    (ui - 70, max(vi - 18, 16)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)
    else:
        cv2.putText(vis, "no target", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
    return add_stamp(vis)


def add_label(vis, text):
    """在单张对比图的顶部居中写 A/B 标签（避开左上角的 no target 与右下角时间戳）。"""
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1)
    org = (max((vis.shape[1] - tw) // 2, 8), 34)
    cv2.putText(vis, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(vis, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 1, cv2.LINE_AA)
    return vis


def hstack_pair(left, right):
    """用 np.hstack 把同一对比的两张图水平拼成一张（尺寸不一致时先统一高度）。"""
    if left.shape[0] != right.shape[0]:
        h = min(left.shape[0], right.shape[0])
        left = cv2.resize(left, (int(left.shape[1] * h / left.shape[0]), h))
        right = cv2.resize(right, (int(right.shape[1] * h / right.shape[0]), h))
    return np.hstack([left, right])


# 对 3 张测试图跑检测并保存标注结果（任务 2 的可视化要求）
def visualize_results():
    print("=== 任务2：检测结果可视化（3 张测试图） ===\n")
    for name, cfg in CONFIG.items():
        img = load_image(os.path.join(IMG_DIR, name))
        if img is None:
            print(f"[错误] 读取失败: {rel_path(os.path.join(IMG_DIR, name))}")
            continue
        detected, u, v, area, mask, cands = detect(img, cfg["ranges"], cfg["min_area"])
        vis = annotate(img, detected, u, v, area, cands)
        stem = os.path.splitext(name)[0]
        imwrite(os.path.join(OUT_DIR, f"{stem}_result.png"), vis)
        state = f"u={u:.1f}, v={v:.1f}" if detected else "未检测到目标"
        print(f"[{name}] {state}  ->  {rel_path(os.path.join(OUT_DIR, stem + '_result.png'))}")


def run_case(img, ranges, min_area=3000, use_morphology=True, label=""):
    """跑一种参数组合，打印统计并返回标注图。"""
    detected, u, v, area, mask, cands = detect(
        img, ranges, min_area, use_morphology=use_morphology)
    ratio = float((mask > 0).mean()) * 100
    print(f"    {label}detected={detected} u={u:.1f} v={v:.1f} area={area:.0f} "
          f"白占比={ratio:.2f}% 候选数={len(cands)} "
          f"候选面积={[int(c[0]) for c in cands[:3]]}")
    return annotate(img, detected, u, v, area, cands)


def save_combined(name, left, right, left_label, right_label):
    """把 A/B 两张图拼在一起保存，并打印相对路径。"""
    combined = hstack_pair(add_label(left, left_label), add_label(right, right_label))
    path = os.path.join(OUT_DIR, name)
    imwrite(path, combined)
    print(f"    ->  {rel_path(path)}  ({combined.shape[1]}x{combined.shape[0]})")


def compare_params():
    """参数对比实验（用 obj3.jpg 黑色圆柱：深色目标靠亮度 V 与其它区域分开）。

    obj3 特点（实测）：目标 V≈35~75，浅色地面 V≈200+，木门 V≈123+。
    所以 V 上限决定“把多少深色区域当成目标”，V 下限决定“多亮的部分还算目标”；
    每个对比只改一个参数，A/B 两张图用 np.hstack 拼成一张保存。
    """
    img = load_image(os.path.join(IMG_DIR, "obj3.jpg"))
    if img is None:
        print(f"[错误] 读取失败: {rel_path(os.path.join(IMG_DIR, 'obj3.jpg'))}")
        return

    base_ranges = [((0, 0, 0), (179, 255, 100))]  # 标定后的默认区间
    base_min_area = 3000

    # 对比 1-上半：HSV 亮度 V 上限（只改 V 上限）
    print("\n对比1（V 上限，100 → 160）：误检")
    vis_a = run_case(img, [((0, 0, 0), (179, 255, 100))], base_min_area)
    vis_b = run_case(img, [((0, 0, 0), (179, 255, 160))], base_min_area)
    save_combined("cmp1_upper_combined.png", vis_a, vis_b, "A: V<=100", "B: V<=160")

    # 对比 1-下半：HSV 亮度 V 下限（只改 V 下限）
    print("对比1（V 下限，0 → 80）：欠分割")
    vis_a = run_case(img, [((0, 0, 0), (179, 255, 100))], base_min_area)
    vis_b = run_case(img, [((0, 0, 80), (179, 255, 100))], base_min_area)
    save_combined("cmp1_lower_combined.png", vis_a, vis_b, "A: V>=0", "B: V>=80")

    # 对比 2：最小轮廓面积 min_area（只改 min_area）—— 噪声过滤
    print("对比2（min_area，500 → 3000）：噪声过滤")
    vis_a = run_case(img, base_ranges, 500)
    vis_b = run_case(img, base_ranges, base_min_area)
    save_combined("cmp2_combined.png", vis_a, vis_b, "A: min_area=500", "B: min_area=3000")

    # 对比 3：是否使用形态学开运算 / 闭运算（其余参数相同）
    print("对比3（不做 / 做开闭运算）：边缘毛刺与孔洞")
    vis_a = run_case(img, base_ranges, base_min_area, use_morphology=False)
    vis_b = run_case(img, base_ranges, base_min_area, use_morphology=True)
    save_combined("cmp3_combined.png", vis_a, vis_b, "A: no open/close", "B: open+close")

    # 对比 4：同一目标（obj2 黄色胶带）在原光照与聚光灯近照下，阈值完全相同
    extra = os.path.join(os.path.dirname(IMG_DIR), "extra", "obj2_light.jpg")
    img_obj2 = load_image(os.path.join(IMG_DIR, "obj2.jpg"))
    img_light = load_image(extra)
    print("对比4（obj2 原光照 vs 聚光灯近照，同一套阈值）：面积与中心随光照/距离变化")
    if img_obj2 is None or img_light is None:
        print(f"    [错误] 读取失败: obj2.jpg 或 {rel_path(extra)}")
        return
    cfg = CONFIG["obj2.jpg"]
    vis_a = run_case(img_obj2, cfg["ranges"], cfg["min_area"], label="A 原光照: ")
    vis_b = run_case(img_light, cfg["ranges"], cfg["min_area"], label="B 暖光版: ")
    save_combined("cmp4_light_combined.png", vis_a, vis_b,
                  "A: obj2 normal light", "B: obj2 spotlight")
    print("    （光照更亮、目标更近：area 46318→52864、中心 v 303.6→338.6，仍只有 1 个候选；"
          "像素坐标与面积不能跨光照直接比较）")


def main():
    if not os.path.isdir(IMG_DIR):
        sys.exit(f"[错误] 找不到 {rel_path(IMG_DIR)}（数据目录应与 src/ 同级）")
    os.makedirs(OUT_DIR, exist_ok=True)
    visualize_results()
    print("\n=== 任务2：参数对比（每次只改一个参数，用 obj3.jpg） ===")
    compare_params()
    print("\n完成，结果在:", rel_path(OUT_DIR))


if __name__ == "__main__":
    main()
