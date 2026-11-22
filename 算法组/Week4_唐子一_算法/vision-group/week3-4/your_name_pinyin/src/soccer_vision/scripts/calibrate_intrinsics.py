#!/usr/bin/env python3
"""相机内参标定（棋盘格）。

考核要求（细则 2.5）：
  - 至少 15 张有效标定图像，覆盖画面中心、四角、不同倾斜角度和距离；
  - 标定分辨率必须与最终推理分辨率一致（1280x720）；
  - 保存 camera_matrix、dist_coeffs、图像宽高、重投影误差。

用法：
    python calibrate_intrinsics.py --images ./calib_images/*.jpg \
        --rows 6 --cols 9 --square 0.025 --width 1280 --height 720 \
        --out calibration/camera_intrinsics.yaml
"""

import argparse
import glob

import cv2
import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", nargs="+", required=True, help="标定图片路径")
    ap.add_argument("--rows", type=int, default=6, help="棋盘格内角点行数")
    ap.add_argument("--cols", type=int, default=9, help="棋盘格内角点列数")
    ap.add_argument("--square", type=float, default=0.025, help="格子边长(米)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--out", default="camera_intrinsics.yaml")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    objpoints, imgpoints = [], []
    img_size = None
    valid = 0
    for path in sorted(args.images):
        img = cv2.imread(path)
        if img is None:
            print(f"[WARN] 无法读取 {path}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = gray.shape[::-1]
        ret, corners = cv2.findChessboardCorners(gray, pattern, None)
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners)
            valid += 1
        else:
            print(f"[WARN] 未检测到棋盘格: {path}")

    if valid < 15:
        print(f"[FAIL] 有效标定图仅 {valid} 张，考核要求至少 15 张")
        return 1

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None
    )
    print(f"标定完成，重投影误差 RMSE = {ret:.4f} px")

    # 写 YAML
    data = {
        "camera_matrix": K.tolist(),
        "dist_coeffs": dist.ravel().tolist(),  # [k1,k2,p1,p2,k3]
        "width": img_size[0],
        "height": img_size[1],
        "reproj_error": float(ret),
        "fx": float(K[0, 0]),
        "fy": float(K[1, 1]),
        "cx": float(K[0, 2]),
        "cy": float(K[1, 2]),
    }
    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"已保存到 {args.out}")
    print(f"  fx={K[0,0]:.2f} fy={K[1,1]:.2f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
    print(f"  dist={dist.ravel().tolist()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
