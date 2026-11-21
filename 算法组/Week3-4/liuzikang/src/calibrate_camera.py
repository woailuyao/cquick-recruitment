#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机内参 + 畸变标定（棋盘格）。"""
from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import cv2
import numpy as np


def find_corners(gray: np.ndarray, cols: int, rows: int):
    """先试 SB（更稳），失败再退回经典方法 + 亚像素细化。"""
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(
            gray, (cols, rows),
            flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            return True, corners.reshape(-1, 1, 2)

    ok, corners = cv2.findChessboardCorners(
        gray, (cols, rows),
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def load_bgr(path: str) -> np.ndarray | None:
    data = np.fromfile(path, dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def save_bgr(path: Path, img: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(path.suffix or ".jpg", img,
                           [cv2.IMWRITE_JPEG_QUALITY, 90])
    if ok:
        buf.tofile(str(path))


def main() -> int:
    ap = argparse.ArgumentParser(description="棋盘格相机标定")
    ap.add_argument("--images", nargs="+", required=True, help="图片路径（支持通配）")
    ap.add_argument("--cols", type=int, default=9, help="内角点列数 = 格子列数-1")
    ap.add_argument("--rows", type=int, default=6, help="内角点行数 = 格子行数-1")
    ap.add_argument("--square", type=float, default=25.0, help="格子边长（毫米）")
    ap.add_argument("--out", default="calibration/camera_intrinsics.yaml")
    ap.add_argument("--min-valid", type=int, default=10, help="至少几张判定成功才标定")
    args = ap.parse_args()

    paths: list[str] = []
    for pattern in args.images:
        matched = glob.glob(pattern)
        paths.extend(matched if matched else [pattern])
    paths = sorted(set(paths))
    if not paths:
        print("没有找到图片")
        return 1

    out_dir = Path(args.out).parent
    corner_dir = out_dir / "corners"

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square

    obj_points, img_points = [], []
    used, failed = [], []
    image_size = None

    print(f"内角点规格 {args.cols}x{args.rows}，格子 {args.square} mm，共 {len(paths)} 张图\n")

    for p in paths:
        img = load_bgr(p)
        name = Path(p).name
        if img is None:
            print(f"  [读不到] {name}")
            failed.append(p)
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])

        ok, corners = find_corners(gray, args.cols, args.rows)
        vis = img.copy()
        if ok:
            cv2.drawChessboardCorners(vis, (args.cols, args.rows), corners, ok)
            obj_points.append(objp)
            img_points.append(corners)
            used.append(p)
            print(f"  [OK]   {name}  找到 {len(corners)} 个内角点")
        else:
            print(f"  [失败] {name}  （裁边/反光/模糊/格子数不对？）")
            failed.append(p)
        save_bgr(corner_dir / f"{Path(p).stem}.jpg", vis)

    print(f"\n判定成功 {len(used)} 张 / 共 {len(paths)} 张，失败 {len(failed)} 张")
    print(f"角点可视化：{corner_dir.resolve()}")

    if len(used) < args.min_valid:
        print(f"\n成功数不足 {args.min_valid} 张，先不标定。")
        print("建议：补拍到 15~20 张，板子占画面 1/3，覆盖中心/四角/不同倾斜角。")
        return 0

    print("\n开始标定……")
    flags = cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST
    rms, K, dist, _, _ = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None, flags=flags)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    print(f"\n重投影误差 RMS = {rms:.3f} px   （< 0.5 算合格，> 1.0 建议重拍）")
    print(f"fx={fx:.2f}  fy={fy:.2f}  cx={cx:.2f}  cy={cy:.2f}")
    print(f"畸变 k1={dist[0,0]:.5f} k2={dist[0,1]:.5f} p1={dist[0,2]:.5f} "
          f"p2={dist[0,3]:.5f} k3={dist[0,4]:.5f}")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        f.write("# 相机内参标定结果（由 tools/calibrate_camera.py 生成）\n")
        f.write(f"# 标定图 {len(used)} 张，内角点 {args.cols}x{args.rows}，格子 {args.square} mm\n")
        f.write(f"image_width: {image_size[0]}\n")
        f.write(f"image_height: {image_size[1]}\n")
        f.write(f"reprojection_rms_px: {rms:.4f}\n")
        f.write("camera_matrix:\n")
        for row in K:
            f.write("  [" + ", ".join(f"{v:.6f}" for v in row) + "]\n")
        f.write("dist_coeffs: [" + ", ".join(f"{v:.8f}" for v in dist.ravel()) + "]\n")
        f.write(f"# 顺序：k1, k2, p1, p2, k3\n")
    print(f"\n已保存：{out_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
