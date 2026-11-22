# -*- coding: utf-8 -*-
"""
用实测地图做闭环自检：合成视角 -> 投影到像素 -> 加噪声 -> 反解位姿 -> 比真值

为什么需要
----------
地图坐标填对了不代表能定位。标志如果挤成一小团，位姿约束就很弱：
两个相距 0.5 m 的标志，各差 5 cm，解出来的朝向能差 8°。
这个脚本用**真实的地图文件 + 真实内参 + 假定外参**做闭环，
在没有任何硬件的情况下先量化"这张地图好不好用"。

用法
----
    python scripts/test_map_localization.py
    python scripts/test_map_localization.py --field config/c0_field.yaml \
        --geometry config/c0_field_geometry.yaml

输出
----
    每个测试机位的真值 vs 解算值、误差；
    以及"只用最近的两个标志"这类不利情形下的表现。
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from src.config import load_camera, load_extrinsics, load_field  # noqa: E402
from src.geometry import CameraModel, Extrinsics, transform_points, wrap_deg  # noqa: E402
from src.matching import FieldMatcher, MarkerObs  # noqa: E402

CAMERA_YAML = os.path.join(ROOT, "config", "camera.yaml")


def project_features(field_map, cam, ex, pose, hfov_half=22.2, vfov_half=13.0,
                     margin=0.98):
    """
    把地图特征按给定载体位姿投影到像素，返回可见的那些。
    pose = (x, y, yaw_deg) 载体在地图中的位姿。
    """
    x, y, yaw = pose
    R = np.array([[math.cos(math.radians(yaw)), -math.sin(math.radians(yaw))],
                  [math.sin(math.radians(yaw)), math.cos(math.radians(yaw))]])
    out = []
    for f in field_map.features:
        P_m = np.array([[f.x, f.y]])
        # 地图 -> 载体
        P_b = (R.T @ (P_m - np.array([[x, y]])).T).T
        P_b3 = np.array([[P_b[0, 0], P_b[0, 1], 0.0]])
        if P_b3[0, 0] <= 0.2:
            continue                      # 在相机后方
        P_c = (ex.R.T @ (P_b3 - ex.t.reshape(1, 3)).T).T
        if P_c[0, 2] <= 0.15:
            continue                      # 太近或在地面以下
        uv, ok = cam.project(P_c)
        if not ok[0]:
            continue
        u, v = uv[0]
        # 视场裁剪
        if not (0 <= u < cam.width and 0 <= v < cam.height):
            continue
        out.append((f, u, v))
    return out


def solve_from(field_map, cam, ex, dets, gate=0.6, max_res=0.35):
    """把可见特征当作检测结果，走一遍完整的几何 + 匹配流程。"""
    obs = []
    for i, (f, u, v) in enumerate(dets):
        uv = np.array([[u, v]], dtype=np.float64)
        d_cam = cam.pixel_to_ray_camera(uv)
        o_b, d_b = ex.ray_to_carrier(d_cam)
        pts, ok = ex.ground_intersection(o_b, d_b)
        if not ok[0]:
            continue
        obs.append(MarkerObs(uid=i + 1, type=f.type,
                             x=float(pts[0, 0]), y=float(pts[0, 1]), conf=0.9))
    matcher = FieldMatcher(field_map.features, gate=gate, max_residual=max_res)
    return matcher.solve(obs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "c0_field.yaml"))
    ap.add_argument("--geometry",
                    default=os.path.join(ROOT, "config", "c0_field_geometry.yaml"))
    ap.add_argument("--height", type=float, default=1.30)
    ap.add_argument("--pitch", type=float, default=-20.0)
    ap.add_argument("--noise", type=float, default=0.0,
                    help="给像素坐标加的高斯噪声标准差（像素）")
    ap.add_argument("--trials", type=int, default=200, help="每个机位重复次数（噪声下）")
    args = ap.parse_args()

    fm = load_field(args.field, args.geometry)
    cam = load_camera(CAMERA_YAML)
    ex = Extrinsics.from_mount(height_m=args.height, pitch_deg=args.pitch)

    print("=" * 74)
    print(f"地图: {os.path.basename(args.field)}   特征 {len(fm.features)} 个")
    by_type = {}
    for f in fm.features:
        by_type.setdefault(f.type, []).append(f)
    for t, lst in by_type.items():
        print(f"  {t:<13} {len(lst)} 个")
    print(f"外参(假定): height={args.height} m, pitch={args.pitch}°")
    print(f"像素噪声: {args.noise} px" + (f"，每组 {args.trials} 次" if args.noise > 0 else ""))

    # 机位：都在场地中线附近，朝向球门（+X）
    poses = [
        (-4.0, 0.0, 0.0),
        (-3.0, 0.0, 0.0),
        (-2.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (-3.0, 0.0, 10.0),
        (-3.0, 0.0, -10.0),
    ]

    print("\n" + "=" * 74)
    hdr = f'{"真值位姿 (x, y, yaw)":<28}{"可见":>5}{"成功":>6}'
    hdr += f'{"平均位置误差":>13}{"平均朝向误差":>13}{"状态":>16}'
    print(hdr)
    print("-" * 74)

    rng = np.random.default_rng(0)
    all_ok = True
    for (px, py, pyaw) in poses:
        base = project_features(fm, cam, ex, (px, py, pyaw))
        n_vis = len(base)
        if n_vis < 2:
            print(f"{f'({px:+.1f}, {py:+.1f}, {pyaw:+.1f}°)':<28}{n_vis:>5}"
                  f"{'—':>6}{'可见标志不足':>28}")
            all_ok = False
            continue

        errs_pos, errs_yaw, n_ok = [], [], 0
        statuses = []
        trials = args.trials if args.noise > 0 else 1
        for _ in range(trials):
            dets = base
            if args.noise > 0:
                dets = [(f, u + rng.normal(0, args.noise), v + rng.normal(0, args.noise))
                        for (f, u, v) in base]
            res = solve_from(fm, cam, ex, dets)
            statuses.append(res.status)
            if res.valid:
                n_ok += 1
                errs_pos.append(math.hypot(res.x - px, res.y - py))
                errs_yaw.append(abs(wrap_deg(res.yaw - pyaw)))

        st = "/".join(sorted(set(statuses)))[:15]
        if n_ok:
            ep = float(np.mean(errs_pos))
            ey = float(np.mean(errs_yaw))
            print(f"{f'({px:+.1f}, {py:+.1f}, {pyaw:+.1f}°)':<28}{n_vis:>5}"
                  f"{n_ok:>4}/{trials:<2}{ep:>12.3f}m{ey:>12.2f}°{st:>16}")
            if ep > 0.5 or ey > 3.0:
                all_ok = False
        else:
            print(f"{f'({px:+.1f}, {py:+.1f}, {pyaw:+.1f}°)':<28}{n_vis:>5}"
                  f"{'0':>4}/{trials:<2}{'—':>13}{'—':>13}{st:>16}")
            all_ok = False

    print("-" * 74)
    print("\n诊断提示：")
    print("  · 可见数 <2 -> 那个位置根本没法定位，需要补测标志")
    print("  · 位置误差 >0.5 m -> 标志太集中，位姿约束弱")
    print("  · 状态出现 ambiguous -> 地图对称性太强，需要不对称标志")
    print("\n结论:", "各机位均可定位 ✅" if all_ok else "存在无法定位或误差偏大的机位 ⚠️")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
