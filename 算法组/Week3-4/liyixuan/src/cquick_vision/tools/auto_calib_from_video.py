#!/usr/bin/env python3
"""从已录好的棋盘格视频 / 图片自动标定内参 —— 不需要相机。

为什么可以不要相机：内参只跟"相机+镜头+分辨率"有关，跟什么时候标无关。
只要素材是同一台相机、同一个分辨率拍的，就能标出正确内参。
（fx/fy/cx/cy 与棋盘格方格真实边长无关，方格边长只影响外参的平移量。）

用法：
  python tools/auto_calib_from_video.py --video chessboard.avi
  python tools/auto_calib_from_video.py --images chess_images --cols 9 --rows 6
  python tools/auto_calib_from_video.py --images chess_images --out calibration/camera_intrinsics.yaml
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2                          # noqa: E402
import numpy as np                  # noqa: E402

from cvlib.config import save_yaml  # noqa: E402

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")


def find_chess(gray, cols, rows):
    try:
        ok, corners = cv2.findChessboardCornersSB(
            gray, (cols, rows), flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
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


def pose_key(corners):
    """位姿指纹，用来去掉同一摆放的重复帧。"""
    c = corners.reshape(-1, 2)
    cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
    x0, y0 = c[0]
    x1, y1 = c[-1]
    ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
    return (round(cx / 25), round(cy / 25), round(ang / 6.0),
            round(float(np.ptp(c[:, 0]) * np.ptp(c[:, 1])) / 20000))


def collect_from_video(path, cols, rows, every, max_views):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f"[error] 打不开视频 {path}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    print(f"[calib] 视频 {os.path.basename(path)} frames={total} 扫描中 ...")
    op, ip, keys = [], [], set()
    size, idx = None, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % every == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            size = (gray.shape[1], gray.shape[0])
            found, corners = find_chess(gray, cols, rows)
            if found and len(op) < max_views:
                k = pose_key(corners)
                if k not in keys:
                    keys.add(k)
                    op.append(None)
                    ip.append(corners)
        idx += 1
    cap.release()
    return op, ip, size


def collect_from_images(folder, cols, rows, max_views):
    files = sorted(p for p in glob.glob(os.path.join(folder, "*"))
                   if p.lower().endswith(IMG_EXT))
    if not files:
        raise SystemExit(f"[error] {folder} 里没有图片")
    print(f"[calib] 图片 {folder} 共 {len(files)} 张，检测中 ...")
    op, ip, keys, size = [], [], set(), None
    for p in files:
        im = cv2.imread(p)
        if im is None:
            continue
        size = (im.shape[1], im.shape[0])
        found, corners = find_chess(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), cols, rows)
        if found and len(op) < max_views:
            k = pose_key(corners)
            if k not in keys:
                keys.add(k)
                op.append(None)
                ip.append(corners)
    return op, ip, size


def main() -> int:
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--video")
    src.add_argument("--images")
    ap.add_argument("--cols", type=int, default=9, help="棋盘格内角点数（列）")
    ap.add_argument("--rows", type=int, default=6, help="棋盘格内角点数（行）")
    ap.add_argument("--square", type=float, default=0.025, help="方格边长 m（只影响外参）")
    ap.add_argument("--every", type=int, default=5, help="视频里每几帧检测一次")
    ap.add_argument("--max-views", type=int, default=80)
    ap.add_argument("--out", default=os.path.join(ROOT, "calibration",
                                                 "camera_intrinsics.yaml"))
    args = ap.parse_args()

    if args.video:
        op, ip, size = collect_from_video(args.video, args.cols, args.rows,
                                          args.every, args.max_views)
        src_name = os.path.basename(args.video)
    else:
        op, ip, size = collect_from_images(args.images, args.cols, args.rows,
                                           args.max_views)
        src_name = os.path.basename(os.path.normpath(args.images))

    if size is None or len(op) < 6:
        print(f"[error] 只找到 {len(op)} 个有效摆位（<6），标定不可靠。")
        print("        建议：棋盘子粘在硬卡纸上压平，录 30~40 秒，"
              "把棋盘格倾斜 30~60 度、覆盖画面四角、远近变化")
        return 3

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_points = [objp.copy() for _ in ip]

    def calibrate(o, i, flags):
        return cv2.calibrateCamera(o, i, size, None, None, flags=flags)

    # 只拟合 k1,k2（k3 在数据质量一般时会把畸变拟合成怪值）
    rms, K, dist, rvecs, tvecs = calibrate(obj_points, ip, cv2.CALIB_FIX_K3)
    per = []
    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        per.append(float(np.mean(np.linalg.norm(
            proj.reshape(-1, 2) - ip[i].reshape(-1, 2), axis=1))))

    print(f"[calib] {len(op)} 个摆位：rms={rms:.3f}px  "
          f"单帧 中位{np.median(per):.2f} 最大{max(per):.2f} px")

    # 剔除离群摆位（最多去掉 30%）
    if len(op) >= 10:
        thr = max(2.5 * float(np.median(per)), 0.8)
        keep = [i for i, e in enumerate(per) if e <= thr]
        if len(keep) >= 8 and len(keep) < len(op):
            n_drop = len(op) - len(keep)
            obj_points = [obj_points[i] for i in keep]
            ip = [ip[i] for i in keep]
            rms, K, dist, rvecs, tvecs = calibrate(obj_points, ip, cv2.CALIB_FIX_K3)
            print(f"[calib] 剔除 {n_drop} 个离群摆位后：rms={rms:.3f}px "
                  f"（剩 {len(ip)} 个）")

    W, H = size
    fov_h = 2.0 * math.degrees(math.atan(W / (2.0 * K[0, 0])))
    fov_v = 2.0 * math.degrees(math.atan(H / (2.0 * K[1, 1])))
    print(f"\n[calib] 图像 {W}x{H}")
    print(f"[calib] fx={K[0,0]:.2f} fy={K[1,1]:.2f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
    print(f"[calib] 视场角 水平 {fov_h:.1f}°  垂直 {fov_v:.1f}°")
    print(f"[calib] 畸变 [k1,k2,p1,p2,k3] = {np.round(dist.ravel(), 5)}")
    print(f"[calib] 重投影误差 {rms:.3f} px")

    # 合理性检查
    ok = True
    if not (20.0 <= fov_h <= 130.0):
        print(f"[warn] 水平视场角 {fov_h:.1f}° 不正常 -> 这次标定不可信")
        ok = False
    if abs(K[0, 2] - W / 2) > 0.25 * W:
        print(f"[warn] cx={K[0,2]:.1f} 离图像中心 {W/2:.0f} 太远 -> 可能检测到了错误的图案")
        ok = False
    if abs(K[1, 2] - H / 2) > 0.25 * H:
        print(f"[warn] cy={K[1,2]:.1f} 离图像中心 {H/2:.0f} 太远 -> 可能检测到了错误的图案")
        ok = False
    if rms > 1.0:
        print("[warn] 重投影误差偏大，最常见原因是棋盘格是软纸、被手拿着弯曲了。"
              "粘到硬卡纸上重录可以把误差降到 0.5px 以内")
    if ok and rms <= 1.0:
        print("[ok] 内参可信")
    elif ok:
        print("[ok] 内参基本可用（误差偏大但可用），想更准就重录棋盘格")

    data = {
        "calibrated": bool(ok),
        "image_width": int(W),
        "image_height": int(H),
        "camera_matrix": [[float(v) for v in row] for row in K.tolist()],
        "dist_coeffs": [float(v) for v in dist.ravel()],
        "rms_reprojection_error_px": float(rms),
        "calib_images": len(ip),
        "square_size_m": float(args.square),
        "board_cols": args.cols,
        "board_rows": args.rows,
        "source": src_name,
        "method": "findChessboardCornersSB + calibrateCamera(CALIB_FIX_K3)",
        "note": "OpenCV dist order [k1,k2,p1,p2,k3]; fx/fy independent of square size",
    }
    save_yaml(args.out, data)
    print(f"[calib] 已写入 {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
