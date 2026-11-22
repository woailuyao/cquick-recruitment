"""
场地标志坐标 · 几何自洽核对（check_field_theory）
=================================================
只做"理论坐标是否与实测几何一致"的静态核对，不调用相机/模型：
  1. 类别计数：L 12 / T 10 / X 2，合计 24；
  2. 边界越界：|x| <= 半长、|y| <= 半宽；
  3. 尺寸换算：禁区/球门区内沿 x、半宽是否与 geometry 尺寸一致；
  4. 对称性：除中线上的点外，±X、±Y 方向应成对出现。

运行（Python 3.10 全路径）：
    python check_field_theory.py
"""
import os
import yaml

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(BASE, "config")

# 类别前缀 -> 期望数量
EXPECT_COUNT = {"LCross": 12, "TCross": 10, "XCross": 2}


def load_yaml(name):
    with open(os.path.join(CFG, name), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    geo = load_yaml("field_geometry.yaml")
    theory = load_yaml("field_theory.yaml")

    # ---- 从 geometry 取尺寸基准（米）----
    f = geo["field"]
    half_len = f["length_x"] / 2.0          # 半长 3.9
    half_wid = f["width_y"] / 2.0          # 半宽 3.0
    box = f["penalty_area"]                # 禁区
    ga = f["goal_area"]                     # 球门区
    exp_box_x = round(half_len - box["length_x"], 3)   # 禁区内沿 x=2.00
    exp_box_y = round(box["width_y"] / 2.0, 3)         # 禁区半宽 1.98
    exp_ga_x = round(half_len - ga["length_x"], 3)     # 球门区内沿 x=2.96
    exp_ga_y = round(ga["width_y"] / 2.0, 3)           # 球门区半宽 1.48

    print("几何基准：半长", half_len, " 半宽", half_wid)
    print(f"  禁区内沿 x=±{exp_box_x} 半宽=±{exp_box_y}")
    print(f"  球门区内沿 x=±{exp_ga_x} 半宽=±{exp_ga_y}\n")

    # ---- 按类别收集点 ----
    by_cls = {k: {} for k in EXPECT_COUNT}
    errors = []
    for key, val in theory.items():
        if key == "map_version" or key == "PenalityPoint":
            continue  # 版本号 / 点球点单独核对
        if not isinstance(val, list) or len(val) != 2:
            continue  # 跳过非坐标键
        x, y = float(val[0]), float(val[1])
        # 取类别前缀（去掉数字序号）
        cls = "".join(ch for ch in key if ch.isalpha())
        if cls not in by_cls:
            errors.append(f"未知类别键: {key}")
            continue
        by_cls[cls][key] = (x, y)

    # ---- 1. 类别计数 ----
    for cls, pts in by_cls.items():
        n = len(pts)
        tag = "OK" if n == EXPECT_COUNT[cls] else "⚠ 数量不符"
        print(f"[计数] {cls}: {n} 个（期望 {EXPECT_COUNT[cls]}）{tag}")
        if n != EXPECT_COUNT[cls]:
            errors.append(f"{cls} 数量 {n} != {EXPECT_COUNT[cls]}")

    # ---- 2. 越界检查 ----
    for cls, pts in by_cls.items():
        for name, (x, y) in pts.items():
            if abs(x) > half_len + 1e-6 or abs(y) > half_wid + 1e-6:
                errors.append(f"越界 {name}: ({x}, {y})")

    # ---- 3. 关键坐标换算核对 ----
    def match(name, x, y, ex, ey):
        return abs(x - ex) < 1e-3 and abs(y - ey) < 1e-3

    for name, (x, y) in by_cls["LCross"].items():
        # 禁区内沿 L5-8：x=±2.00, y=±1.98；球门区 L9-12：x=±2.96, y=±1.48
        if abs(abs(x) - exp_box_x) < 1e-3 and abs(abs(y) - exp_box_y) < 1e-3:
            continue
        if abs(abs(x) - exp_ga_x) < 1e-3 and abs(abs(y) - exp_ga_y) < 1e-3:
            continue
        if abs(abs(x) - half_len) < 1e-3 and abs(abs(y) - half_wid) < 1e-3:
            continue  # 外框角
        errors.append(f"L 点坐标与几何不符: {name} ({x}, {y})")

    # ---- 4. 对称性：把所有点装进集合，核对 ±X/±Y 配对 ----
    all_pts = {(round(x, 3), round(y, 3))
               for pts in by_cls.values() for x, y in pts.values()}
    for x, y in all_pts:
        # 中线上 x=0 或 y=0 的点只核对另一方向配对
        if x != 0 and (round(-x, 3), y) not in all_pts:
            errors.append(f"缺 ±X 对称点: ({x}, {y})")
        if y != 0 and (x, round(-y, 3)) not in all_pts:
            errors.append(f"缺 ±Y 对称点: ({x}, {y})")

    # ---- 点球点单独核对 ----
    pp = theory.get("PenalityPoint")
    if pp != [0.0, 0.0]:
        errors.append(f"点球点应为 [0,0]，当前 {pp}")

    print("\n==== 核对结果 ====")
    if errors:
        print("发现问题：")
        for e in errors:
            print("  ⚠", e)
    else:
        print("全部通过：24 点计数、边界、尺寸换算、对称性一致。")


if __name__ == "__main__":
    main()
