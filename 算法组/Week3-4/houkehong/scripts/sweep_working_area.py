# -*- coding: utf-8 -*-
"""
扫描"哪些相机位置能稳定定位"，给现场摆三脚架用

做法：在网格上遍历相机位姿，用真实地图 + 真实内参 + 假定外参做闭环
（投影 -> 加噪声 -> 反解），看每个位置能不能解出正确位姿。

用法：
    python scripts/sweep_working_area.py
    python scripts/sweep_working_area.py --height 1.32 --pitch -19.8 --noise 2
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from scripts.test_map_localization import project_features, solve_from  # noqa: E402
from src.config import load_camera, load_field  # noqa: E402
from src.geometry import Extrinsics  # noqa: E402

CAMERA_YAML = os.path.join(ROOT, "config", "camera.yaml")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "c0_field.yaml"))
    ap.add_argument("--geometry",
                    default=os.path.join(ROOT, "config", "c0_field_geometry.yaml"))
    ap.add_argument("--height", type=float, default=1.30)
    ap.add_argument("--pitch", type=float, default=-20.0)
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--tol", type=float, default=0.5, help="算作成功的最大位置误差（米）")
    args = ap.parse_args()

    fm = load_field(args.field, args.geometry)
    cam = load_camera(CAMERA_YAML)
    ex = Extrinsics.from_mount(height_m=args.height, pitch_deg=args.pitch)
    rng = np.random.default_rng(0)

    ys = [1.0, 0.5, 0.0, -0.5, -1.0]
    xs = [2.0, 1.5, 1.0, 0.5, 0.0, -0.5, -1.0, -1.5, -2.0,
          -2.5, -3.0, -3.5, -4.0, -5.0, -6.0, -7.0]

    print(f"地图 {os.path.basename(args.field)}，外参 h={args.height} pitch={args.pitch}°，"
          f"噪声 {args.noise} px")
    print("行 = 相机 X（越负 = 离球门越远），列 = 横向偏移 Y\n")
    print("    X \\ Y " + "".join(f"{y:>7.1f}" for y in ys))
    print("  " + "-" * (9 + 7 * len(ys)))

    n_ok = n_all = 0
    for x in xs:
        row = f"  {x:>6.1f} "
        for y in ys:
            base = project_features(fm, cam, ex, (x, y, 0.0))
            if len(base) < 2:
                row += f"{'.':>7}"
                n_all += 1
                continue
            trials = 12 if args.noise > 0 else 1
            ok = 0
            amb = 0
            for _ in range(trials):
                dets = base
                if args.noise > 0:
                    dets = [(f, u + rng.normal(0, args.noise), v + rng.normal(0, args.noise))
                            for (f, u, v) in base]
                res = solve_from(fm, cam, ex, dets)
                if res.valid and math.hypot(res.x - x, res.y - y) < args.tol:
                    ok += 1
                elif res.status == "ambiguous":
                    amb += 1
            n_all += 1
            if ok == trials:
                row += f"{'*':>7}"
                n_ok += 1
            elif ok > 0:
                row += f"{'~':>7}"       # 有时成功
                n_ok += 1
            elif amb >= trials / 2:
                row += f"{'a':>7}"
            else:
                row += f"{'x':>7}"
        print(row)

    print("\n* 稳定定位   ~ 时好时坏   a 歧义（标志不足或对称）   x 解错   . 可见标志 <2")
    print(f"\n可定位机位: {n_ok}/{n_all}")
    print("建议：三脚架放在能稳定定位的区域内，且让画面里同时有 3 个以上标志。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
