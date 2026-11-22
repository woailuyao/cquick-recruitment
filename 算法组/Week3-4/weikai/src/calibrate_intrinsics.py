# -*- coding: utf-8 -*-
"""calibrate_intrinsics.py —— 相机内参标定（任务二 solvePnP 的前置）

输入：D:\\captures\\calib\\ 里的棋盘格照片（board_*.png）
输出：D:\\captures\\calib\\intrinsics.yaml
      （相机矩阵 K、畸变系数 dist、重投影 RMS、每张图的误差明细）

用法（先拿尺子量一下打印出来的棋盘格实际边长，把实测值传进来）：
  python calibrate_intrinsics.py --square-mm 20.0
  python calibrate_intrinsics.py --square-mm 19.8 --show   # 逐张显示检测效果
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

# 棋盘规格：10x7 方格 -> 9x6 内角点（与 make_calibration_board.py 默认一致）
PATTERN = (9, 6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=r"D:\captures\calib")
    ap.add_argument("--square-mm", type=float, default=20.0,
                    help="方格实际边长（毫米），务必用尺子量打印件")
    ap.add_argument("--show", action="store_true",
                    help="逐张显示角点检测结果（按任意键下一张）")
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.dir, "*.png")) +
                   glob.glob(os.path.join(args.dir, "*.jpg")))
    if len(paths) < 10:
        print("图片太少（%d 张），至少 10 张、建议 15-25 张。" % len(paths))
        return 1

    square_m = args.square_mm / 1000.0
    objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2)
    objp *= square_m

    objpoints, imgpoints = [], []
    used, skipped, per_img_err = [], [], []
    gray_size = None

    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print("[跳过] 读不了:", os.path.basename(p))
            skipped.append((os.path.basename(p), "读不了"))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_size = gray.shape[::-1]
        # 优先用 SB（基于棋盘扇区，对轻微模糊/倾斜鲁棒），失败再退回经典算法
        ok, corners = cv2.findChessboardCornersSB(gray, PATTERN)
        method = "SB"
        if not ok:
            ok, corners = cv2.findChessboardCorners(
                gray, PATTERN,
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)
            method = "classic"
        if not ok:
            print("[跳过] 未找到角点:", os.path.basename(p))
            skipped.append((os.path.basename(p), "未找到角点"))
            continue
        if method == "classic":
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        objpoints.append(objp)
        imgpoints.append(corners)
        used.append(os.path.basename(p))
        if args.show:
            vis = img.copy()
            cv2.drawChessboardCorners(vis, PATTERN, corners, ok)
            cv2.imshow("corners (any key = next, ESC = stop showing)", vis)
            if cv2.waitKey(0) & 0xFF == 27:
                args.show = False
        print("[OK] %s  角点 %d" % (os.path.basename(p), len(corners)))

    if args.show:
        cv2.destroyAllWindows()

    print("-" * 60)
    print("有效图片 %d / 共 %d 张" % (len(used), len(paths)))
    if len(used) < 8:
        print("有效图片不足 8 张，标定不可靠；建议补拍（角度多样化）。")
        return 1

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray_size, None, None)

    # 每张图的重投影误差
    total_err = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i],
                                    K, dist)
        err = float(np.linalg.norm(
            imgpoints[i].reshape(-1, 2) - proj.reshape(-1, 2),
            axis=1).mean())
        per_img_err.append((used[i], err))
        total_err.append(err)
    total_err = np.array(total_err)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    fov_x = 2 * np.degrees(np.arctan2(gray_size[0] / 2.0, fx))
    fov_y = 2 * np.degrees(np.arctan2(gray_size[1] / 2.0, fy))

    print("RMS 重投影误差: %.4f px（<0.5 很好，<1.0 可用）" % rms)
    print("平均单图误差  : %.4f px（最差 %.4f px）"
          % (total_err.mean(), total_err.max()))
    print("相机矩阵 K:")
    print("  fx = %.1f  fy = %.1f  cx = %.1f  cy = %.1f" % (fx, fy, cx, cy))
    print("畸变系数 dist:", np.round(dist.ravel(), 6).tolist())
    print("视场角: %.1f° x %.1f°（分辨率 %dx%d）"
          % (fov_x, fov_y, *gray_size))
    worst = sorted(per_img_err, key=lambda e: -e[1])[:3]
    print("误差最大的 3 张（可考虑剔除后重标）:",
          ", ".join("%s %.2fpx" % (n, e) for n, e in worst))

    out = os.path.join(args.dir, "intrinsics.yaml")
    fs = cv2.FileStorage(out, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", gray_size[0])
    fs.write("image_height", gray_size[1])
    fs.write("camera_matrix", K)
    fs.write("distortion_coefficients", dist)
    fs.write("rms", rms)
    fs.write("square_mm", args.square_mm)
    fs.write("pattern", "%dx%d" % PATTERN)
    fs.write("n_images", len(used))
    fs.release()
    print("-" * 60)
    print("已保存:", out)
    print("下一步：用它做 solvePnP（定位）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
