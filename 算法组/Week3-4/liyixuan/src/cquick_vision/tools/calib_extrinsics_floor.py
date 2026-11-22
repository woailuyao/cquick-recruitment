#!/usr/bin/env python3
"""外参标定：由"平放在地面的棋盘格"求出相机相对地面参考系的位姿。

关键设计（不依赖棋盘格在地面上的摆放方向）：
  1. 棋盘格平放在地面 -> 它所在的平面就是地面，其法向就是"上"的方向；
  2. 由 solvePnP 得到棋盘格在相机系的位姿，取法向并校正符号，
     得到世界"上"方向 up（相机系表示），以及相机到地面的垂直距离 h；
  3. 定义载体系 base：
       原点 = 相机光心在地面上的垂直投影点
       +X   = 相机光轴在地面上的水平投影方向
       +Y   = up × X（右手系）
       +Z   = up
     于是相机在 base 下的位置恒为 (0, 0, h)，滚转按定义为零。
     这对应细则允许的"把相机光心定义为载体参考原点"。
  4. 地面投影只依赖 R_base_camera 与 t_base_camera，与棋盘格朝向无关，
     所以随便怎么摆、只要平就行。

用法：
  python tools\\calib_extrinsics_floor.py --camera 1 --cols 9 --rows 6 --square 0.025
  空格=采集，q=完成并写入
"""
from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2                          # noqa: E402
import numpy as np                  # noqa: E402

from cvlib.camera import open_capture                 # noqa: E402
from cvlib.config import load_intrinsics, save_yaml  # noqa: E402


def find_corners(gray, cols, rows):
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


def base_from_board_pose(R_cb: np.ndarray, t_cb: np.ndarray):
    """由棋盘格位姿构造 R_base_camera / t_base_camera 及几何量。

    R_cb, t_cb:  P_cam = R_cb @ P_board + t_cb
    返回 dict，含 R_base_cam, t_base_cam, height, pitch_deg, roll_deg
    """
    # 相机中心在棋盘格坐标系中的位置
    C_b = -R_cb.T @ t_cb.reshape(3)
    h = abs(float(C_b[2]))
    s = 1.0 if C_b[2] > 0 else -1.0
    up = s * R_cb[:, 2]                    # 世界上方向（相机系表示）
    up = up / np.linalg.norm(up)

    fwd = np.array([0.0, 0.0, 1.0])        # 相机光轴（相机系）
    fwd_h = fwd - float(np.dot(fwd, up)) * up
    n = np.linalg.norm(fwd_h)
    if n < 1e-6:
        raise SystemExit("[error] 相机光轴几乎垂直于地面，检查相机是不是朝天/朝地")
    X = fwd_h / n
    Y = np.cross(up, X)
    Z = up
    R_base_cam = np.vstack([X, Y, Z])      # 每行是一个 base 轴在相机系中的方向
    # 校验右手系
    if np.linalg.det(R_base_cam) < 0:
        R_base_cam = np.vstack([X, -Y, Z])
    t_base_cam = np.array([0.0, 0.0, h])   # 相机在 base 下的位置：(0,0,+h)

    fwd_base = R_base_cam @ fwd            # 光轴在 base 下的方向
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -float(fwd_base[2])))))
    roll = 0.0                             # 按定义为零（X 轴由光轴水平投影定义）
    return {"R_base_cam": R_base_cam, "t_base_cam": t_base_cam,
            "height": h, "pitch_deg": pitch, "roll_deg": roll,
            "up_cam": up, "board_center_in_cam": t_cb.reshape(3),
            "camera_in_board": C_b}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="1")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.025)
    ap.add_argument("--intrinsics", default=os.path.join(
        ROOT, "calibration", "camera_intrinsics.yaml"))
    ap.add_argument("--out", default=os.path.join(
        ROOT, "calibration", "camera_extrinsics.yaml"))
    ap.add_argument("--image", default=None, help="直接用一张图片标定，不开相机")
    args = ap.parse_args()

    intr = load_intrinsics(args.intrinsics)
    K, dist = intr["K"], intr["dist"]
    if not intr["calibrated"]:
        print("[warn] 内参文件标记为未标定，请先跑 auto_calib_from_video.py")

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    shots = []

    def solve_and_report(corners, tag):
        okp, rvec, tvec = cv2.solvePnP(
            objp, corners.reshape(-1, 1, 2).astype(np.float64), K, dist,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not okp:
            print("[warn] solvePnP 失败")
            return None
        proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
        err = float(np.mean(np.linalg.norm(
            proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)))
        R_cb, _ = cv2.Rodrigues(rvec)
        m = base_from_board_pose(R_cb, tvec)
        print(f"[extr] {tag} reproj={err:.3f}px  高度={m['height']:.4f} m  "
              f"俯角={m['pitch_deg']:.2f} deg")
        return {"corners": corners, "R_cb": R_cb, "t_cb": tvec.reshape(3),
                "err": err, "m": m}

    if args.image:
        im = cv2.imread(args.image)
        if im is None:
            print(f"[error] 读不了 {args.image}")
            return 2
        found, corners = find_corners(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY),
                                      args.cols, args.rows)
        if not found:
            print("[error] 这张图里找不到完整棋盘格")
            return 3
        r = solve_and_report(corners, "image")
        if r:
            shots.append(r)
    else:
        src = int(args.camera) if str(args.camera).isdigit() else args.camera
        cap = open_capture(src, intr["width"], intr["height"])
        if cap is None:
            print("[error] 打不开相机")
            return 2
        print("把棋盘格平放在地上（朝着哪个方向都行，只要平），空格采集，q 结束")
        print("采 3~5 张，每张之间把板子挪动一点点")
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_corners(gray, args.cols, args.rows)
            vis = frame.copy()
            if found:
                cv2.drawChessboardCorners(
                    vis, (args.cols, args.rows),
                    corners.reshape(-1, 1, 2).astype(np.float32), True)
            cv2.putText(vis, f"shots={len(shots)} board={'OK' if found else '--'}",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if found else (0, 0, 255), 2)
            cv2.imshow("calib_extrinsics_floor", vis)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == 32 and found:
                r = solve_and_report(corners, f"shot #{len(shots)+1}")
                if r:
                    shots.append(r)
        cap.release()
        cv2.destroyAllWindows()

    if not shots:
        print("[error] 没有采到有效图像")
        return 3

    best = min(shots, key=lambda s: s["err"])
    m = best["m"]
    R_base_cam = m["R_base_cam"]
    t_base_cam = m["t_base_cam"]

    print(f"\n[extr] 用 {len(shots)} 张中重投影误差最小的一张（{best['err']:.3f}px）")
    print(f"[extr] 相机高度 h = {m['height']:.4f} m   俯角 = {m['pitch_deg']:.2f} deg")
    if len(shots) > 1:
        hs = [s["m"]["height"] for s in shots]
        ps = [s["m"]["pitch_deg"] for s in shots]
        print(f"[extr] 多张一致性：高度 {min(hs):.3f}~{max(hs):.3f} m，"
              f"俯角 {min(ps):.2f}~{max(ps):.2f} deg")
        if max(hs) - min(hs) > 0.02:
            print("[warn] 高度离散 >2cm：相机与载具有相对位移，或板子没放平")
    print(f"[extr] R_base_camera =\n{np.array2string(R_base_cam, precision=5)}")
    print(f"[extr] t_base_camera = {np.round(t_base_cam, 4)}")

    # 验证：把棋盘格中心与四角投影到 base，z 应≈0
    print("[extr] 自检（棋盘格各角的 base 坐标，z 应≈0）：")
    worst = 0.0
    for idx in (0, args.cols - 1, len(objp) - 1, len(objp) - args.cols):
        Pc = best["R_cb"] @ objp[idx].reshape(3) + best["t_cb"]
        Pb = R_base_cam @ Pc + t_base_cam
        worst = max(worst, abs(float(Pb[2])))
        print(f"   board[{idx:2d}] -> base {np.round(Pb, 4)}")
    if worst > 0.02:
        print(f"[warn] 角点离地最大 {worst*100:.1f} cm，说明板子不平或标定有误差")
    else:
        print(f"[ok] 角点离地最大 {worst*100:.1f} cm，几何自洽")

    data = {
        "calibrated": True,
        "frame": "camera_ground_projection",
        "camera_frame": "camera_optical",
        "image_width": intr["width"],
        "image_height": intr["height"],
        "rotation": [[float(v) for v in row] for row in R_base_cam.tolist()],
        "translation": [float(v) for v in t_base_cam.tolist()],
        "camera_height_m": float(m["height"]),
        "pitch_deg_down": float(m["pitch_deg"]),
        "roll_deg": 0.0,
        "camera_yaw_offset_deg": 0.0,
        "rms_reprojection_error_px": float(best["err"]),
        "method": "floor chessboard solvePnP -> ground-plane normal defines up; "
                  "base origin = camera ground projection, +X = camera forward projected",
        "shots": len(shots),
        "convention": "P_base = rotation @ P_camera + translation; "
                      "base: +Z up (from board normal), +X = optical axis horizontal projection",
        "note": "base 原点定义为相机光心在地面的投影（细则允许以相机光心为参考原点）",
    }
    save_yaml(args.out, data)
    print(f"\n[extr] 已写入 {args.out}")
    print("[extr] 下一步：python tools\\verify_ground.py --camera 1  用卷尺核对 2/3/5 m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
