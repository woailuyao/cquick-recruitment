# -*- coding: utf-8 -*-
"""相机外参自标定：用场地标志把「桌面支架上的相机」姿态直接解出来。

背景（2026-11-08 口径）：相机固定在**桌沿支架**上，外参若靠量角器量，
俯仰角误差 2° 就会带来数分米的地面误差。更好的做法是：
相机架好、拍一张能同时看到 ≥4 个已知标志的照片，用 PnP 直接解出
相机相对场地的完整姿态（高度 / 俯仰 / 滚转 / 朝向），再与卷尺量到的高度互校。

对应表格里「完成相机相对于机器人底座 / 移动测试架的外参标定（12 分）」。

两种用法：

1) 实测模式（有相机、有照片后有像素坐标）：
       python tools/estimate_extrinsics.py --from-file calibration/extrinsics_correspondences.yaml

2) 自检模式（无相机，用仿真验证方法本身正确）：
       python tools/estimate_extrinsics.py --sim

对应文件格式见 calibration/extrinsics_correspondences.example.yaml
"""

from __future__ import annotations

import argparse
import math
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import load_yaml, pick, save_yaml              # noqa: E402
from src.common.paths import CALIB_DIR, rel                             # noqa: E402
from src.loc.camera_model import CameraModel, Extrinsics, ground_to_pixels  # noqa: E402
from src.loc.planar_pose import (                                       # noqa: E402
    check_extrinsics_consistency, planar_pose_to_extrinsics, solve_planar_pose,
)


def _sim_correspondences():
    """用仿真的相机参数生成一组像素坐标（自检用，不依赖任何真实文件）。"""
    cam = CameraModel(fx=650.0, fy=650.0, cx=640.0, cy=360.0,
                      dist=(-0.15, 0.05, 0.0008, -0.0004, -0.008), width=1280, height=720)
    ext_truth = Extrinsics.from_params(height=0.62, pitch_deg=21.0, roll_deg=0.8)
    ext_truth.t[0:2] = 0.0
    # 相机站在场地上某处，朝球门方向略偏
    pose = (-1.2, 0.6, math.radians(28.0))
    truth_markers = [
        ("LCross", 7.00, 4.50), ("LCross", 7.00, -4.50),
        ("TCross", 0.00, 4.50), ("XCross", 0.00, 0.00),
        ("LCross", 5.50, 2.00), ("PenalityPoint", 5.00, 0.00),
    ]
    rows = []
    for t, mx, my in truth_markers:
        dx, dy = mx - pose[0], my - pose[1]
        c, s = math.cos(-pose[2]), math.sin(-pose[2])
        bx, by = c * dx - s * dy, s * dx + c * dy      # 地图 -> 载体
        if math.hypot(bx, by) > 12.0:
            continue
        uv, ok = ground_to_pixels(cam, ext_truth, [(bx, by, 0.0)])
        if not ok[0]:
            continue
        rows.append({"type": t, "x": mx, "y": my,
                     "u": round(float(uv[0][0]), 3), "v": round(float(uv[0][1]), 3)})
    return cam, rows, pose, ext_truth


def main() -> int:
    ap = argparse.ArgumentParser(description="相机外参自标定（PnP）")
    ap.add_argument("--from-file", default=None, help="对应点文件（YAML）")
    ap.add_argument("--intrinsics", default=str(CALIB_DIR / "camera_intrinsics.yaml"),
                    help="内参文件，默认 calibration/camera_intrinsics.yaml")
    ap.add_argument("--out", default=str(CALIB_DIR / "camera_extrinsics.yaml"),
                    help="外参输出文件")
    ap.add_argument("--sim", action="store_true", help="仿真自检模式（不需要相机）")
    ap.add_argument("--ransac-px", type=float, default=3.0, help="RANSAC 重投影阈值")
    args = ap.parse_args()

    print("=" * 72)
    print("相机外参自标定 · PnP")
    print("=" * 72)

    if args.sim:
        cam, rows, pose, ext_truth = _sim_correspondences()
        tape_height = 0.62
        print("模式       : 仿真自检（无相机）")
        print(f"真值位姿   : x={pose[0]:.2f} y={pose[1]:.2f} yaw={math.degrees(pose[2]):.1f}°")
        print(f"真值外参   : 高度 {ext_truth.t[2]:.3f} m，俯角 21.0°，滚转 0.8°")
        out_path = None
    else:
        if not args.from_file:
            print("错误：实测模式必须用 --from-file 指定对应点文件（或用 --sim 自检）")
            return 1
        data = load_yaml(args.from_file)
        cam = CameraModel.from_yaml(args.intrinsics)
        rows = pick(data, "correspondences", "points", default=[])
        tape_height = pick(data.get("measurement", {}), "tape_height_m", default=None)
        print(f"模式       : 实测（{rel(args.from_file)}）")
        print(f"内参       : {rel(args.intrinsics)}")
        print(f"对应点数   : {len(rows)}")
        out_path = Path(args.out)
        if not out_path.is_absolute():
            out_path = ROOT / out_path

    if len(rows) < 4:
        print(f"错误：至少需要 4 个对应点（单应要求共面四点非共线），当前 {len(rows)} 个")
        return 1

    map_points = [(float(r["x"]), float(r["y"])) for r in rows]
    pixels = [(float(r["u"]), float(r["v"])) for r in rows]
    res = solve_planar_pose(map_points, pixels, cam, reproj_thresh_px=args.ransac_px)

    print("-" * 72)
    if not res.valid:
        print(f"求解失败 : status={res.status}，内点 {res.n_inliers}/{res.n_points}")
        print("可能原因 : 标志点共线 / 内参不对 / 像素坐标与地图类别不匹配 / 点是镜像解")
        return 1

    print(f"求解结果 : 内点 {res.n_inliers}/{res.n_points}，重投影 RMS {res.rms_px:.3f} px")
    print(f"相机位置 : x={res.x:.3f} m  y={res.y:.3f} m  (地图坐标系)")
    print(f"相机姿态 : yaw={res.yaw_deg:.2f}°  俯角={res.pitch_deg:.2f}°  滚转={res.roll_deg:.2f}°")
    print(f"光心高度 : {res.height_m:.3f} m")

    ext = planar_pose_to_extrinsics(res)
    check = check_extrinsics_consistency(res)
    print(f"姿态互校 : 参数化还原差 {check['rotation_diff_deg']:.4f}°（应接近 0）")

    if tape_height:
        diff = abs(float(tape_height) - res.height_m)
        verdict = "一致" if diff <= 0.05 else "偏差偏大，请复核"
        print(f"卷尺互校 : 卷尺 {float(tape_height):.3f} m vs PnP {res.height_m:.3f} m "
              f"→ 差 {diff:.3f} m（{verdict}）")

    if args.sim:
        err_xy = math.hypot(res.x - pose[0], res.y - pose[1])
        err_yaw = abs(math.degrees((res.yaw_deg * math.pi / 180 - pose[2] + math.pi)
                                   % (2 * math.pi) - math.pi))
        print(f"位姿误差 : 位置 {err_xy * 100:.2f} cm，航向 {err_yaw:.3f}°")
        ok = err_xy < 0.02 and err_yaw < 0.5 and res.rms_px < 1.0 and check["ok"]
        print(f"自检结论 : {'通过' if ok else '未通过'}")
        print("=" * 72)
        if not ok:
            return 1
        out_path = CALIB_DIR / "camera_extrinsics.simcheck.yaml"
        print(f"仿真结果另存: {rel(out_path)}（不覆盖真实外参文件）")

    payload = {
        "camera_extrinsics": {
            "frame": "camera_frame" if args.sim else pick(
                load_yaml(args.from_file).get("frame", {}), "name", default="camera_frame"),
            "camera_height_m": round(res.height_m, 4),
            "pitch_deg": round(res.pitch_deg, 3),
            "roll_deg": round(res.roll_deg, 3),
            "yaw_offset_deg": 0.0,
            "forward_offset_m": 0.0,
            "lateral_offset_m": 0.0,
            "method": "PnP 自标定（场地标志对应点 + 内参），solvePnPRansac + LM 精化",
            "reprojection_rms_px": round(res.rms_px, 4),
            "inliers": f"{res.n_inliers}/{res.n_points}",
            "source": rel(args.from_file) if args.from_file else "sim",
            "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "tape_height_m": tape_height,
        }
    }
    save_yaml(out_path, payload)
    print("-" * 72)
    print(f"已写出外参 : {rel(out_path)}")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
