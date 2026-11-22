# -*- coding: utf-8 -*-
"""棋盘格相机内参标定，写出 calibration/camera_intrinsics.yaml。

本次提交的内参就是用本脚本在外接 USB 摄像头上标出来的：
    11/15 张有效图，整体 RMS 0.699 px，fx=fy=1564.285, cx=639.5, cy=359.5

如果现场必须换相机（例如改用手机 / 另一个摄像头），一定要重新跑一遍，
否则内参与实际镜头不符，地面投影会整体偏。

用法:
    python calibrate_intrinsics.py --images D:/vision_data/calibration/corners
    python calibrate_intrinsics.py --images ... --cols 9 --rows 9 --square 0.0317
    python calibrate_intrinsics.py --images ... --free  # 不固定主点/方像素

注意:
    cols/rows 是“内角点”数量，不是方格数量。
    --square 单位米，只影响平移量，不影响 fx/fy/cx/cy。
"""

import argparse
import glob
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(HERE, "..", "calibration", "camera_intrinsics.yaml")


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="标定图目录")
    ap.add_argument("--cols", type=int, default=9, help="内角点列数")
    ap.add_argument("--rows", type=int, default=9, help="内角点行数")
    ap.add_argument("--square", type=float, default=0.0317, help="方格边长 m")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--free", action="store_true",
                    help="自由估计 k1/k2/p1/p2/k3 与主点；默认固定主点+方像素+只估 k1")
    ap.add_argument("--min-images", type=int, default=8)
    return ap.parse_args()


def main():
    args = parse_args()
    pattern = (args.cols, args.rows)
    files = sorted(glob.glob(os.path.join(args.images, "*.jpg")) +
                   glob.glob(os.path.join(args.images, "*.png")))
    if not files:
        raise SystemExit(f"目录里没有图片: {args.images}")

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    obj_points, img_points = [], []
    size = None
    for path in files:
        img = cv2.imread(path)
        if img is None:
            continue
        if size is None:
            size = (img.shape[1], img.shape[0])
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCornersSB(
            gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if not ok:
            ok, corners = cv2.findChessboardCorners(gray, pattern)
            if ok:
                corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        if not ok:
            print("未检出:", os.path.basename(path))
            continue
        obj_points.append(objp)
        img_points.append(corners)
        print("检出:", os.path.basename(path))

    if len(obj_points) < args.min_images:
        raise SystemExit(f"有效图只有 {len(obj_points)} 张，至少需要 {args.min_images} 张")

    w, h = size
    k0 = np.array([[float(w), 0, (w - 1) / 2.0],
                   [0, float(w), (h - 1) / 2.0],
                   [0, 0, 1]], dtype=np.float64)
    if args.free:
        flags = 0
        k0 = None
        mode = "自由估计"
    else:
        flags = (cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO |
                 cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
        mode = "固定主点 + 方像素 + 只估 k1"

    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, size, k0, None, flags=flags)
    print(f"有效图 {len(obj_points)}/{len(files)}，模式: {mode}")
    print(f"RMS {rms:.4f} px  fx={K[0,0]:.3f} fy={K[1,1]:.3f} "
          f"cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
    print("dist:", np.round(D.ravel(), 6).tolist())

    text = f"""# 单目相机内参（{mode}）
# 由 src/calibrate_intrinsics.py 生成
image_width: {w}
image_height: {h}
camera_matrix:
  rows: 3
  cols: 3
  data: [{K[0,0]:.6f}, 0.000000, {K[0,2]:.6f}, 0.000000, {K[1,1]:.6f}, {K[1,2]:.6f}, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
dist_coeffs: [{D.ravel()[0]:.8f}, {D.ravel()[1] if D.size > 1 else 0.0:.8f}, {D.ravel()[2] if D.size > 2 else 0.0:.8f}, {D.ravel()[3] if D.size > 3 else 0.0:.8f}, {D.ravel()[4] if D.size > 4 else 0.0:.8f}]
rms_reprojection_error: {rms:.6f}
checkerboard: {args.cols}x{args.rows} inner corners
square_size_mm: {args.square * 1000:.1f}
calibration_images_used: {len(obj_points)}
calibration_images_total: {len(files)}
"""
    if not args.out:
        print(text)
        return
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print("已写入", out)


if __name__ == "__main__":
    main()