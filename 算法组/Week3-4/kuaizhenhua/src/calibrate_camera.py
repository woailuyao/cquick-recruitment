#!/usr/bin/env python3
"""用棋盘格照片标定 1280x720 单目相机，并写出普通 YAML。"""
from __future__ import annotations
import argparse
from pathlib import Path
import cv2
import numpy as np
import yaml


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--input-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--cols", type=int, default=9, help="棋盘格内角点列数")
    p.add_argument("--rows", type=int, default=6, help="棋盘格内角点行数")
    p.add_argument("--square-size", type=float, required=True, help="棋盘格单格边长，米")
    args = p.parse_args()
    files = sorted([*args.input_dir.glob("*.jpg"), *args.input_dir.glob("*.jpeg"), *args.input_dir.glob("*.png")])
    if len(files) < 15:
        raise SystemExit(f"标定照片至少准备 15 张，当前只有 {len(files)} 张。")
    obj = np.zeros((args.rows * args.cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_size
    object_points, image_points = [], []
    image_size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    for path in files:
        image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_size = (gray.shape[1], gray.shape[0])
        if image_size is not None and current_size != image_size:
            raise ValueError('Calibration images must all use the same resolution')
        image_size = current_size
        ok, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, .001))
        else:
            ok, corners = cv2.findChessboardCornersSB(gray, (args.cols, args.rows), cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            object_points.append(obj.copy())
            image_points.append(corners)
    if len(image_points) < 10:
        raise SystemExit(f"有效棋盘格照片只有 {len(image_points)} 张，建议重新拍摄不同角度和距离。")
    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(object_points, image_points, image_size, None, None)
    result = {
        "status": "provisional_from_chessboard",
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "reprojection_error_px": float(rms),
        "calibration_date": __import__('datetime').datetime.now().astimezone().isoformat(),
        "source_images": len(image_points),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"saved={args.output} valid_images={len(image_points)} resolution={image_size[0]}x{image_size[1]} reprojection_error_px={rms:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
