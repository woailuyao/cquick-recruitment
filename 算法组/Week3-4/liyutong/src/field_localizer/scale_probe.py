# -*- coding: utf-8 -*-
"""
尺度判决：用已知尺寸的棋盘格验证「内参 + PnP 解出来的距离」是不是真米。

原理：内参定死之后，棋盘格角点的像素坐标 + 已知格边长 20 mm
      → solvePnP 直接解出棋盘中心在相机坐标系下的 (X, Y, Z)，单位米。
      Z 就是相机到棋盘的直线距离。此时不需要任何外参、不需要地面假设。
      如果卷尺量的是 1.500 m 而 PnP 读出 1.28 m，说明整条链路短了 15%。

用法：
    python scale_probe.py videos/scale_7.mp4 1.500
    python scale_probe.py scale_imgs             # 目录也行
    python scale_probe.py videos/scale_7.mp4     # 不给真值，只看读数稳不稳
"""
import os
import sys
import glob
import numpy as np
import cv2

BOARD = (9, 6)          # 内角点数（列, 行）
SQUARE_MM = 20.0
CAL_NPZ = "calibration.npz"

objp = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * SQUARE_MM
# 棋盘中心在棋盘坐标系里的位置（用于报告"板中心"距离）
center_3d = np.array([[(BOARD[0] - 1) * SQUARE_MM / 2,
                       (BOARD[1] - 1) * SQUARE_MM / 2, 0.0]], np.float32)


def load_cal():
    if not os.path.exists(CAL_NPZ):
        sys.exit(f"找不到 {CAL_NPZ}，先跑 calib_compute.py")
    d = np.load(CAL_NPZ)
    K = d["K"] if "K" in d else d["mtx"]
    dist = d["dist"] if "dist" in d else d["dist_coeffs"]
    return K.astype(np.float64), dist.astype(np.float64)


def imread_safe(p):
    buf = np.fromfile(p, np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR) if buf.size else None


def solve_frame(frame, K, dist):
    """返回 (板中心距离 m, 板中心 XYZ m, 角点重投影 RMS px)，失败返回 None"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    ok, corners = cv2.findChessboardCorners(gray, BOARD, flags)
    if not ok:
        ok, corners = cv2.findChessboardCornersSB(gray, BOARD)
        if not ok:
            return None
        corners = corners.reshape(-1, 1, 2).astype(np.float32)
    else:
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    c3, _ = cv2.projectPoints(center_3d, rvec, tvec, K, dist)
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1))))

    R, _ = cv2.Rodrigues(rvec)
    center_cam = (R @ center_3d.reshape(3, 1) + tvec).reshape(3)
    return float(np.linalg.norm(center_cam)), center_cam, rms


def collect(path):
    """把视频/目录里的所有帧解一遍，返回 [(tag, dist_m, center, rms, frame)]"""
    out = []
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "*.*")))
        for p in files:
            if os.path.splitext(p)[1].lower() not in (".jpg", ".jpeg", ".png", ".bmp"):
                continue
            img = imread_safe(p)
            if img is not None:
                out.append((os.path.basename(p), img))
    else:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            sys.exit(f"打不开 {path}")
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
        print(f"{os.path.basename(path)}: {n} 帧, {fps:.1f} fps, 约 {n / fps:.1f} s")
        step = max(1, n // 60)          # 最多取 60 帧，够统计了
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if i % step == 0:
                out.append((f"frame{i}", frame))
            i += 1
        cap.release()
    return out


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    truth = float(sys.argv[2]) if len(sys.argv) > 2 else None

    K, dist = load_cal()
    print(f"内参: fx={K[0,0]:.2f} fy={K[1,1]:.2f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}")

    frames = collect(path)
    hits = []
    for tag, img in frames:
        r = solve_frame(img, K, dist)
        if r:
            hits.append((tag, *r))

    print(f"\n检出棋盘 {len(hits)}/{len(frames)} 帧")
    if not hits:
        sys.exit("一帧都没检出：棋盘是否完整入画？光照是否够？换 findChessboardCornersSB 也不行？")

    d = np.array([h[1] for h in hits])
    rms = np.array([h[3] for h in hits])
    z = np.array([h[2][2] for h in hits])
    print(f"距离: 中位 {np.median(d):.4f} m  均值 {d.mean():.4f}  标准差 {d.std() * 1000:.1f} mm  "
          f"范围 [{d.min():.3f}, {d.max():.3f}]")
    print(f"重投影 RMS: 中位 {np.median(rms):.2f} px（<1.0 才算解算可信）")
    print(f"相机系 Z 中位 {np.median(z):.4f} m（相机水平看时 ≈ 距离）")

    if truth:
        ratio = np.median(d) / truth
        print(f"\n真值 {truth:.3f} m → PnP/真值 = {ratio:.4f}")
        err = (ratio - 1) * 100
        print(f"尺度误差 {err:+.1f}%")
        if abs(err) < 2:
            print("→ 内参尺度正确。外参 0.45 / 几何 0.39 的分歧出在'地面平面法'那一步，"
                  "以内参为准重解外参即可。")
        elif 10 < abs(err) < 25:
            print("→ 内参链路有系统性尺度偏差。典型原因：打印棋盘时被'适应页面'缩放了。"
                  "量一下打印纸上实际格边长，填回 SQUARE_MM 重测。")
        else:
            print("→ 误差不在 15% 量级，说明真值或棋盘边长两者之一填错，先核对。")

        # 反推"如果 PnP 是对的，棋盘真实格边长应该是多少"
        implied = SQUARE_MM * truth / np.median(d)
        print(f"\n反推：若相机解算可信，则实际格边长 = {implied:.2f} mm（设计值 {SQUARE_MM} mm）。"
              f"拿尺子量打印纸上 5 个格子的总长（应 {5 * implied:.1f} mm）即可一锤定音。")


if __name__ == "__main__":
    main()
