# -*- coding: utf-8 -*-
"""相机外参标定（相机 -> 载体），写出 calibration/camera_extrinsics.yaml。

只标定“相机相对地面的姿态”：高度 h、俯仰 pitch（往下看为正）、滚转 roll。
航向 yaw 不写进外参，由定位模块给出，避免重复计数。
载体原点定义为相机光心在地面上的垂直投影，所以平移固定为 (0, 0, h)。

两种方式
--------
1) 手工测量（最快，现场推荐先做这个）
   python calibrate_extrinsics.py --mode manual --height 0.58 --pitch-down 12 --roll 0

   高度用卷尺量相机镜头中心到地面；pitch 用手机水平仪 App 贴在相机上读
   （镜头朝下时读数为“俯角”），roll 同理。没有角度计时，先填 0，
   再用 4 个以上已知标志做 PnP 修正。

2) PnP（更准，需要至少 4 个互不共线的已知标志）
   python calibrate_extrinsics.py --mode pnp --points ../docs/extrinsic_points.yaml

   points 文件格式（pixel 用原图未去畸变像素，map 用 field.yaml 坐标）:
       points:
         - type: TCross
           pixel: [640.0, 480.0]
           map: [3.470, -2.965]
         ...

   注意：PnP 只取姿态和高度，不取航向；因此站在场地哪个位置都能标。
   标完后用 --dry-run 先看一眼数值，合理再写文件。
"""

import argparse
import datetime
import math
import os

import cv2
import numpy as np

import minyaml
from ground_geometry import CameraModel, rotation_from_angles

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_INTR = os.path.join(HERE, "..", "calibration", "camera_intrinsics.yaml")
DEFAULT_OUT = os.path.join(HERE, "..", "calibration", "camera_extrinsics.yaml")


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["manual", "pnp"], required=True)
    ap.add_argument("--height", type=float, help="manual: 相机光心离地高度 m")
    ap.add_argument("--pitch-down", type=float, default=0.0,
                    help="manual: 俯角，镜头往下看为正，单位度")
    ap.add_argument("--roll", type=float, default=0.0,
                    help="manual: 滚转角，单位度")
    ap.add_argument("--points", help="pnp: 已知标志像素与地图坐标的 yaml/json")
    ap.add_argument("--intrinsics", default=DEFAULT_INTR)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--dry-run", action="store_true", help="只打印，不写文件")
    return ap.parse_args()


def solve_pnp(cam, points_path):
    data = minyaml.load(points_path) or {}
    pts = data.get("points") if isinstance(data, dict) else data
    if not pts or len(pts) < 4:
        raise SystemExit("PnP 至少需要 4 个点（points 列表）")

    obj, img = [], []
    for p in pts:
        obj.append([float(p["map"][0]), float(p["map"][1]), 0.0])
        img.append([float(p["pixel"][0]), float(p["pixel"][1])])
    obj = np.array(obj, dtype=np.float64)
    img = np.array(img, dtype=np.float64)

    ok, rvec, tvec = cv2.solvePnP(obj, img, cam.K, cam.D,
                                  flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise SystemExit("solvePnP 失败，检查点坐标与像素是否对应")
    R, _ = cv2.Rodrigues(rvec)

    # X_cam = R * X_map + t  =>  相机光轴在地图中的方向 = R 的第 3 行
    forward = R[2, :]
    yaw = math.degrees(math.atan2(forward[1], forward[0]))
    pitch = math.degrees(math.asin(float(np.clip(-forward[2], -1.0, 1.0))))
    # 去掉航向后剩下的滚转
    rz = np.array([[math.cos(math.radians(-yaw)), -math.sin(math.radians(-yaw)), 0],
                   [math.sin(math.radians(-yaw)), math.cos(math.radians(-yaw)), 0],
                   [0, 0, 1]])
    # R.T 是 camera->map，去掉航向后应等于 R_bc = Rx(roll) Ry(pitch) R0
    r_level = rz @ R.T
    roll = math.degrees(math.asin(float(np.clip(-r_level[2, 0], -1.0, 1.0))))

    center = -R.T @ tvec.reshape(3)
    height = float(center[2])

    proj, _ = cv2.projectPoints(obj, rvec, tvec, cam.K, cam.D)
    err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)
    rms = float(np.sqrt(np.mean(err ** 2)))
    return height, pitch, roll, yaw, rms, len(pts), center


def write_yaml(path, height, pitch, roll, method, rms, note=""):
    R = rotation_from_angles(pitch, roll)
    rows = "[%s]" % ", ".join("% .6f" % float(v) for v in R.reshape(-1))
    text = f"""# 相机外参（相机 -> 载体）
#
# 坐标系:
#   camera  : OpenCV 光学系，x 右, y 下, z 沿光轴
#   carrier : x 前, y 左, z 上；原点 = 相机光心在地面上的垂直投影
#   平移固定为 [0, 0, h]，因此 carrier 位姿与相机光心的二维地图位姿一致。
#
# 标定方式: {method}
# 标定日期: {datetime.date.today().isoformat()}
# 重投影误差: {f"{rms:.3f} px" if rms is not None else "无（手工测量）"}
# {note}

status: calibrated

frame:
  parent: carrier
  child: camera_optical_frame

translation_m: [0.0, 0.0, {height:.4f}]
rotation_matrix: {rows}
camera_height_m: {height:.4f}
pitch_deg: {pitch:.3f}
roll_deg: {roll:.3f}

calibration_method: {method}
reprojection_error_px: {f"{rms:.3f}" if rms is not None else "null"}
calibration_date: {datetime.date.today().isoformat()}
"""
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def main():
    args = parse_args()
    cam = CameraModel(args.intrinsics)

    if args.mode == "manual":
        if args.height is None:
            raise SystemExit("manual 模式需要 --height（卷尺量相机光心离地高度）")
        height, pitch, roll = args.height, args.pitch_down, args.roll
        rms, n_pts, method = None, 0, "卷尺量高度 + 地面点法/水平仪测安装角"
        if abs(pitch) < 1e-6:
            print("提示: --pitch-down 为 0，地面投影会退化（射线几乎与地面平行），"
                  "请务必填写实际俯角。")
        print(f"高度 {height:.3f} m  俯角 {pitch:.2f}°  滚转 {roll:.2f}°")
    else:
        if not args.points:
            raise SystemExit("pnp 模式需要 --points 文件")
        height, pitch, roll, yaw, rms, n_pts, center = solve_pnp(cam, args.points)
        method = f"已知场地标志 solvePnP（{n_pts} 点，SOLVEPNP_IPPE）"
        print(f"PnP 结果: 高度 {height:.3f} m  俯角 {pitch:.2f}°  "
              f"滚转 {roll:.2f}°  (标定时航向 {yaw:.1f}°，不写入外参)")
        print(f"重投影误差 {rms:.3f} px  相机地图坐标 "
              f"({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")
        if height < 0.05 or height > 2.5:
            print("警告: 高度不在 0.05~2.5 m，检查点坐标/像素是否对应错了")
        if abs(pitch) > 60:
            print("警告: 俯角超过 60°，检查是否有符号或对应错误")

    if args.dry_run:
        print("--dry-run: 不写文件")
        return

    out = os.path.abspath(args.out)
    write_yaml(out, height, pitch, roll, method, rms,
               note="现场如有条件，建议再用 4 点以上做 PnP 复核。")
    print("已写入", out)


if __name__ == "__main__":
    main()