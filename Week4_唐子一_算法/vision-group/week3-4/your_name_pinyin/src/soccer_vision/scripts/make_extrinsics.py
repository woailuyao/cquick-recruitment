#!/usr/bin/env python3
"""生成相机外参模板（camera_extrinsics.yaml）。

考核要求（细则 2.6）：说明相机坐标到载体坐标的旋转矩阵/平移向量、
相机光心高度、安装俯仰角/滚转角、标定方法和单位。
不得把标称高度或手工估计角度伪装成精确标定值——本模板只生成占位，
实际数值请测量后填写。

用法：
    python make_extrinsics.py --height 0.35 --pitch 15 --roll 0 --out calibration/camera_extrinsics.yaml
"""

import argparse

import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, default=0.35, help="相机光心离地高度(米)")
    ap.add_argument("--pitch", type=float, default=15.0, help="俯仰角(度，向下为正)")
    ap.add_argument("--roll", type=float, default=0.0, help="滚转角(度)")
    ap.add_argument("--out", default="camera_extrinsics.yaml")
    args = ap.parse_args()

    # 由 height/pitch/roll 构造 R, t（载体参考原点在相机正下方地面投影）
    #
    # 相机坐标系约定（OpenCV 标准）：+X 向右，+Y 向下，+Z 指向相机前方（光轴方向）。
    # 载体坐标系：+Z 向上，+X 向前（指向球门方向），+Y 向左，地面为 Z=0。
    #
    # 相机"俯视地面"的姿态 = 光轴从水平朝下转 pitch 角。在"相机->载体"变换中，
    # 俯仰是绕【相机 X 轴】旋转（X 轴是左右方向，俯仰绕它转），不是绕 Y 轴。
    # 光轴朝下 pitch 角时，把相机 +Z（朝前下方）转回载体水平方向，需要绕 X 轴
    # 转 +pitch（向下为正）。
    pitch = np.radians(args.pitch)   # 俯仰角，向下为正
    roll = np.radians(args.roll)     # 滚转角
    # 相机 -> 载体：先绕 X 轴转 pitch（俯仰），再绕 Z 轴转 roll（滚转）
    # 注意：这里直接构造"载体坐标 = Rx(pitch) @ Rx(roll) @ 相机坐标"。
    Rx_pitch = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)],
    ])
    Rz_roll = np.array([
        [np.cos(roll), -np.sin(roll), 0],
        [np.sin(roll), np.cos(roll), 0],
        [0, 0, 1],
    ])
    # 相机光轴朝下俯视时，相机 +Z 在载体系里是斜向下的。
    # 要让"相机坐标 -> 载体坐标"正确，R 应为 Rx(pitch)（把俯视姿态转正）。
    # 相机 +Y 向下，+Z 向前：光轴水平时 R=I；光轴向下转 pitch 时，
    # 载体坐标 = Rx(pitch) @ 相机坐标。
    R = Rx_pitch @ Rz_roll
    # 相机光心在载体坐标系中的位置：在原点正上方高度 h
    t = np.array([0.0, 0.0, args.height])

    data = {
        "R": R.tolist(),
        "t": t.tolist(),
        "height": args.height,
        "pitch_deg": args.pitch,
        "roll_deg": args.roll,
        "unit": "meter",
        "note": "外参为相机->载体参考坐标系；数值需实测，不可用标称值冒充精确标定",
        "calibration_method": "手工测量 + 水平仪/量角器（请补充实测方法）",
    }
    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"已生成外参模板 {args.out}")
    print("注意：height/pitch/roll 需按实测值填写，R/t 由本脚本自动构造。")


if __name__ == "__main__":
    main()
