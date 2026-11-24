#!/usr/bin/env python3

import argparse
import glob
import os

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "config", "camera.yaml")

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(ROOT, "data", "calib"))
    ap.add_argument("--pattern", nargs=2, type=int, default=[9, 6],
                    help="棋盘格内角点数：横向 纵向（不是格子数）")
    ap.add_argument("--square", type=float, default=25.0, help="每个方格边长，毫米")
    ap.add_argument("--min", type=int, default=15, help="至少需要多少张成功检测的图")
    ap.add_argument("--fix-k3", action="store_true",
                    help="固定 k3=0。视角不够丰富时能避免畸变系数发散")
    ap.add_argument("--fix-principal", action="store_true",
                    help="固定主点在图像中心。手持数据约束不足时更稳")
    ap.add_argument("--fix-aspect", action="store_true",
                    help="强制 fx=fy（方形像素），进一步减少未知量")
    return ap.parse_args()

def main():
    args = parse_args()
    pattern = tuple(args.pattern)
    images = sorted(glob.glob(os.path.join(args.dir, "*.jpg")) +
                    glob.glob(os.path.join(args.dir, "*.png")))
    print(f"找到 {len(images)} 张图，棋盘内角点 {pattern}，方格 {args.square} mm")
    if not images:
        print("[错误] 目录里没有图片")
        return 1

    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * args.square

    obj_points = []
    img_points = []
    size = None

    for path in images:
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, pattern, None)
        if not ok:
            print(f"  跳过（没找到角点）：{os.path.basename(path)}")
            continue
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        obj_points.append(objp)
        img_points.append(corners)

    print(f"成功检测 {len(obj_points)} 张（需要至少 {args.min} 张）")
    if len(obj_points) < args.min:
        print("[错误] 有效标定图不够，补拍：多角度、多位置、画面四角都要有")
        return 1

    flags = 0
    if args.fix_k3:
        flags |= cv2.CALIB_FIX_K3
    if args.fix_principal:
        flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if args.fix_aspect:
        flags |= cv2.CALIB_FIX_ASPECT_RATIO
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, size, None, None, flags=flags)
    print(f"重投影误差 RMS: {rms:.3f} px（越小越好，一般 < 1.0）")
    print("K =\n", K)
    print("dist =", dist.ravel())

    data = {
        "image_width": int(size[0]),
        "image_height": int(size[1]),
        "camera_matrix": K.tolist(),
        "dist_coeffs": dist.ravel().tolist(),
        "rms": float(rms),
    }
    with open(OUT, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"已写入 {OUT}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
