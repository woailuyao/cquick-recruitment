#!/usr/bin/env python3

import argparse
import math
import os

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "config", "camera_extrinsics.yaml")

def build_R(pitch_deg, roll_deg, yaw_deg):
    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)
    y = math.radians(yaw_deg)

    c, s = math.cos(p), math.sin(p)
    R_pitch = np.array([
        [0.0, -s, c],
        [-1.0, 0.0, 0.0],
        [0.0, -c, -s],
    ])

    R_roll = np.array([
        [math.cos(r), -math.sin(r), 0.0],
        [math.sin(r), math.cos(r), 0.0],
        [0.0, 0.0, 1.0],
    ])

    R_yaw = np.array([
        [math.cos(y), -math.sin(y), 0.0],
        [math.sin(y), math.cos(y), 0.0],
        [0.0, 0.0, 1.0],
    ])

    return R_yaw @ R_pitch @ R_roll

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, required=True, help="镜头中心离地高度（米）")
    ap.add_argument("--pitch", type=float, default=0.0, help="俯仰角（度，向下为正）")
    ap.add_argument("--roll", type=float, default=0.0, help="滚转角（度）")
    ap.add_argument("--yaw", type=float, default=0.0, help="相对载体 +X 的偏航（度，一般 0）")
    ap.add_argument("--x", type=float, default=0.0, help="镜头中心相对载体原点的前向偏移（米）")
    ap.add_argument("--y", type=float, default=0.0, help="镜头中心相对载体原点的左向偏移（米）")
    ap.add_argument("--method", default="卷尺 + 手机水平仪")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()

    R = build_R(args.pitch, args.roll, args.yaw)
    det = float(np.linalg.det(R))
    ortho = float(np.max(np.abs(R @ R.T - np.eye(3))))

    data = {
        "R_base_camera": [[round(float(v), 8) for v in row] for row in R],
        "t_base_camera": [args.x, args.y, args.height],
        "camera_height_m": args.height,
        "pitch_deg": args.pitch,
        "roll_deg": args.roll,
        "yaw_deg": args.yaw,
        "method": args.method,
        "camera_frame": "+X right, +Y down, +Z forward",
        "base_frame": "+X forward, +Y left, +Z up",
    }
    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    print("已写入", args.out)
    print("det(R) =", round(det, 6), " 正交误差 =", f"{ortho:.2e}")
    print("R =")
    print(np.array2string(R, precision=4, suppress_small=True))
    print("t =", data["t_base_camera"])
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
