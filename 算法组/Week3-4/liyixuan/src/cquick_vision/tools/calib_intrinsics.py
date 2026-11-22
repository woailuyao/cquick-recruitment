#!/usr/bin/env python3
"""棋盘格内参标定，输出 calibration/camera_intrinsics.yaml。

采集要点：必须用最终推理分辨率（默认 1280x720）；覆盖中心/四角/远近/倾斜；
全部角点检测成功才允许采集。重投影误差 > 0.5 px 建议重标。

用法：
  python tools/calib_intrinsics.py --camera 0 --cols 9 --rows 6 --square 0.025
  按键：空格=采集   q=结束并标定   r=清空
"""
from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2                          # noqa: E402
import numpy as np                  # noqa: E402

from cvlib.camera import open_capture  # noqa: E402
from cvlib.config import save_yaml  # noqa: E402


def find_corners(gray, cols, rows):
    try:
        ok, corners = cv2.findChessboardCornersSB(
            gray, (cols, rows),
            flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            return True, corners.reshape(-1, 2).astype(np.float64)
    except AttributeError:
        pass
    ok, corners = cv2.findChessboardCorners(
        gray, (cols, rows),
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return False, None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
    return True, corners.reshape(-1, 2).astype(np.float64)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="0")
    ap.add_argument("--cols", type=int, default=9, help="内角点数（列）")
    ap.add_argument("--rows", type=int, default=6, help="内角点数（行）")
    ap.add_argument("--square", type=float, default=0.025, help="方格边长 m")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--min", type=int, default=15)
    ap.add_argument("--out", default=os.path.join(ROOT, "calibration",
                                                 "camera_intrinsics.yaml"))
    args = ap.parse_args()

    src = int(args.camera) if str(args.camera).isdigit() else args.camera
    cap = open_capture(src, args.width, args.height)
    if cap is None:
        print("[error] 打不开相机", args.camera)
        return 2

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_points, img_points = [], []
    size = None
    last_t = 0.0

    print(f"按空格采集（目标 >= {args.min} 张），q 结束并标定，r 清空")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        found, corners = find_corners(gray, args.cols, args.rows)

        vis = frame.copy()
        if found:
            cv2.drawChessboardCorners(vis, (args.cols, args.rows),
                                      corners.reshape(-1, 1, 2).astype(np.float32), True)
        cv2.putText(vis, f"captured {len(obj_points)}/{args.min}  "
                         f"board={'OK' if found else '--'}  {size[0]}x{size[1]}",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.imshow("calib_intrinsics", vis)

        k = cv2.waitKey(1) & 0xFF
        now = time.time()
        auto = (k == 32) or (found and now - last_t > 0.0 and False)
        if k == ord("q"):
            break
        if k == ord("r"):
            obj_points.clear()
            img_points.clear()
            print("[calib] cleared")
            continue
        if auto and found:
            obj_points.append(objp.copy())
            img_points.append(corners.astype(np.float64))
            last_t = now
            print(f"[calib] captured #{len(obj_points)}")

    cap.release()
    cv2.destroyAllWindows()

    if len(obj_points) < 4:
        print(f"[error] 有效图像不足（{len(obj_points)} 张），至少 4 张，建议 15 张以上")
        return 3

    print(f"[calib] 标定中：{len(obj_points)} 张，{size[0]}x{size[1]} ...")
    rms, K, dist, _, _ = cv2.calibrateCamera(
        obj_points, img_points, size, None, None,
        flags=cv2.CALIB_FIX_K3 if len(obj_points) < 10 else 0)
    print(f"[calib] rms reprojection error = {rms:.4f} px")
    print("[calib] K =\n", np.array2string(K, precision=4))
    print("[calib] dist =", np.round(dist.ravel(), 6))
    if rms > 0.5:
        print("[warn] 重投影误差偏大：检查棋盘格是否平整、图像是否模糊、是否用了软纸打印")

    data = {
        "calibrated": True,
        "image_width": int(size[0]),
        "image_height": int(size[1]),
        "camera_matrix": [[float(v) for v in row] for row in K.tolist()],
        "dist_coeffs": [float(v) for v in dist.ravel()[:5]]
        if len(dist.ravel()) >= 5 else [float(v) for v in dist.ravel()],
        "rms_reprojection_error_px": float(rms),
        "calib_images": len(obj_points),
        "square_size_m": float(args.square),
        "board_cols": args.cols,
        "board_rows": args.rows,
        "note": "OpenCV calibrateCamera, dist order [k1,k2,p1,p2,k3]",
    }
    save_yaml(args.out, data)
    print("[calib] saved", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
