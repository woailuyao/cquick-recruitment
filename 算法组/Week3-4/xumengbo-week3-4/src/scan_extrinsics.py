"""
外参参数扫描（scan_extrinsics）
================================
只在内存里试算，不写回 camera_extrinsics.yaml。
在当前外参附近小范围扫描 pitch / z / yaw / y，对现场三点 MARKERS
计算平面误差，按平均误差排序输出最优组合，供现场微调方向参考。

注意：三点拟合存在过拟合风险，扫描结果只指示方向，最终外参仍以
现场实测读数为准；本脚本不修改任何配置文件。

运行：python scan_extrinsics.py
"""
import os
import sys
import math
import itertools

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import loc_common
import ground_projection

# 现场标记：(名称, 真实X, 真实Y, 像素u, 像素v)，与 check_extrinsics_projection.py 一致
MARKERS = [
    ("P1", -0.5, 0.5, 497, 626),
    ("P2",  0.5, 0.0, 668, 531),
    ("P3", -1.0,-0.5, 945, 648),
]

# 基准值（当前 yaml）
BASE = {
    "x": -3.87, "y": -0.10, "z": 0.73,
    "pitch": -5.0, "roll": 4.07, "yaw": -4.0,
}

# 扫描网格（度 / 米）；roll 与 x 平移先固定基准值
PITCH_GRID = [-8.0, -6.0, -5.0, -4.0, -2.0]
Z_GRID     = [0.68, 0.70, 0.73, 0.76, 0.79]
YAW_GRID   = [-8.0, -6.0, -4.0, -2.0, 0.0]
Y_GRID     = [-0.30, -0.20, -0.10, 0.00, 0.10]


def evaluate(intr, pitch, z, yaw, y):
    """用给定外参试算三点，返回 (平均误差cm, 最大误差cm, 有效点数, 各点误差列表)。"""
    ext = loc_common.Extrinsics(
        translation=[BASE["x"], y, z],
        rotation_deg={"pitch": pitch, "roll": BASE["roll"], "yaw": yaw},
        measured=True,
        orientation_base="forward",
    )
    proj = ground_projection.GroundProjector(intr, ext, field_half=(3.9, 3.0))
    errs = []
    for name, xt, yt, u, v in MARKERS:
        res = proj.project_one(u, v)
        if not res["valid"]:
            errs.append(None)
            continue
        errs.append(math.hypot(res["x_b"] - xt, res["y_b"] - yt))
    valid = [e for e in errs if e is not None]
    if not valid:
        return float("inf"), float("inf"), 0, errs
    mean_err = sum(valid) / len(valid)
    max_err = max(valid)
    return mean_err * 100, max_err * 100, len(valid), errs


def main():
    intr = loc_common.Intrinsics.from_config()

    # 先打印基准组
    m0, x0, n0, e0 = evaluate(intr, BASE["pitch"], BASE["z"], BASE["yaw"], BASE["y"])
    print("=== 当前基准外参 ===")
    print(f"pitch={BASE['pitch']} z={BASE['z']} yaw={BASE['yaw']} y={BASE['y']}")
    print(f"平均 {m0:.1f} cm；最大 {x0:.1f} cm；有效 {n0}/3")
    for (name, *_), e in zip(MARKERS, e0):
        print(f"  {name}: {'无效' if e is None else f'{e*100:.1f} cm'}")

    # 网格扫描
    results = []
    for pitch, z, yaw, y in itertools.product(PITCH_GRID, Z_GRID, YAW_GRID, Y_GRID):
        mean_err, max_err, n, errs = evaluate(intr, pitch, z, yaw, y)
        if n == 3:  # 三点都有效才参与排序
            results.append((mean_err, max_err, pitch, z, yaw, y, errs))

    results.sort(key=lambda r: r[0])
    print("\n=== 平均误差最小的前 10 组（仅试算，未写回 yaml）===")
    print(f"{'排名':<4} {'平均cm':<8} {'最大cm':<8} {'pitch':<7} {'z':<6} {'yaw':<7} {'y':<7} 各点cm")
    for i, (mean_err, max_err, pitch, z, yaw, y, errs) in enumerate(results[:10], 1):
        per = " ".join(f"{e*100:.1f}" if e is not None else "NA" for e in errs)
        print(f"{i:<4} {mean_err:<8.1f} {max_err:<8.1f} {pitch:<7.1f} {z:<6.2f} {yaw:<7.1f} {y:<7.2f} {per}")

    print("\n提示：扫描只给方向；三点拟合可能过拟合，最终以现场实测和更多测试点为准。")


if __name__ == "__main__":
    main()
