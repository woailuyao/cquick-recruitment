#!/usr/bin/env python3
"""不接相机，检查外参"能看到多远的地面"。

外参不对最典型的症状：俯角太大 -> 相机几乎对着脚底，画面里只有半米地面，
远处的场地标志根本进不了视野，定位必然失败。

用法：
  python tools\\check_extrinsics.py
  python tools\\check_extrinsics.py --intrinsics ... --extrinsics ...
"""
from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2                          # noqa: E402
import numpy as np                  # noqa: E402

from cvlib.config import load_extrinsics, load_intrinsics  # noqa: E402
from cvlib.geom import ray_to_ground                      # noqa: E402


def ground_at(u, v, intr, extr):
    pts = np.asarray([[[float(u), float(v)]]], np.float64)
    nx, ny = cv2.undistortPoints(pts, intr["K"], intr["dist"], P=None).reshape(-1, 2)[0]
    d_c = np.array([float(nx), float(ny), 1.0])
    d_c /= np.linalg.norm(d_c)
    d_b = extr["R"] @ d_c
    P, why = ray_to_ground(d_b, extr["t"], max_range=200.0)
    if P is None:
        return None, why
    return P, None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--intrinsics", default=os.path.join(
        ROOT, "calibration", "camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics", default=os.path.join(
        ROOT, "calibration", "camera_extrinsics.yaml"))
    args = ap.parse_args()

    intr = load_intrinsics(args.intrinsics)
    extr = load_extrinsics(args.extrinsics)
    W, H = intr["width"], intr["height"]

    print(f"[extr] 相机高度 h = {extr['height']:.4f} m   "
          f"俯角 = {extr['pitch_deg']:.2f} deg   图像 {W}x{H}")
    print(f"[extr] 垂直视场约 {2*math.degrees(math.atan(H/2/intr['fy'])):.1f} deg\n")

    rows = [("画面最上边", 0.02), ("上 1/4", 0.25), ("画面中心", 0.5),
            ("下 3/4", 0.75), ("画面最下边", 0.98)]
    print("画面不同高度对应的地面距离（相机地面投影 -> 该点）：")
    dists = {}
    for name, frac in rows:
        v = frac * H
        P, why = ground_at(W / 2, v, intr, extr)
        if P is None:
            print(f"  {name:10s} v={v:6.1f}  ->  打不到地面 ({why})")
            continue
        D = math.hypot(float(P[0]), float(P[1]))
        dists[name] = D
        print(f"  {name:10s} v={v:6.1f}  ->  base=({P[0]:6.2f},{P[1]:6.2f})  "
              f"距离 {D:6.2f} m")

    print()
    far = dists.get("画面最上边", 0.0)
    near = dists.get("画面最下边", 0.0)
    ok = True
    if far < 3.0:
        print(f"[FAIL] 画面最上边只能看到 {far:.2f} m —— 太近了，"
              f"远处的场地标志进不了画面，定位做不了")
        print("       原因：相机俯角太大（几乎对着脚底）或装得太高")
        ok = False
    elif far < 4.0:
        print(f"[ok] 画面最上边能看到 {far:.2f} m —— 够用（标志在 {far:.1f} m 内可见）。"
              f"想更宽裕可以再抬 2~3 度，但不是必须")
    elif far > 40:
        print(f"[warn] 画面最上边能看 {far:.1f} m，可能接近地平线，"
              f"远处像素误差会被放大")
    else:
        print(f"[ok] 画面最上边能看到 {far:.2f} m，远处标志可见")

    if near > 2.0:
        print(f"[warn] 画面最下边是 {near:.2f} m，脚边看不到，"
              f"近距离标志可能漏掉")
    else:
        print(f"[ok] 画面最下边 {near:.2f} m，近处也看得到")

    if ok:
        print("\n外参视野合理。下一步：python tools\\verify_ground.py --camera 1 用卷尺核对")
    else:
        print("\n需要重标外参：把相机改成朝前下方 15~25 度、离地 0.4~0.6 m 再标")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
