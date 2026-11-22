# -*- coding: utf-8 -*-
"""
单目相机内参标定（细则 2.5）

两种用法：

  1) 真实标定（相机到货后）
       python scripts/calibrate_camera.py --images calib_images --pattern 9x6 --square 25

  2) 自检（**不需要相机**）
       python scripts/calibrate_camera.py --selftest

     自检会造一个有**已知内参**的虚拟相机，让它拍若干张合成棋盘格，
     再用本脚本的标定流程反解 fx/fy/cx/cy 和畸变系数，
     最后把反解值 vs 真值打出来。
     如果两者吻合，说明标定代码是对的，等相机到了直接拍真实图就能用。

输出 calibration/camera_intrinsics.yaml，并附带重投影误差。
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------- 标定核心

def build_object_points(pattern_cols, pattern_rows, square_mm):
    """棋盘格的内角点 3D 坐标（Z=0 平面）。注意 pattern 是**内角点数**。"""
    objp = np.zeros((pattern_cols * pattern_rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_cols, 0:pattern_rows].T.reshape(-1, 2)
    objp *= float(square_mm) / 1000.0     # 转成米
    return objp


def find_corners(gray, pattern_cols, pattern_rows, use_sb=True):
    """优先用 findChessboardCornersSB（亚像素精度高、对噪声鲁棒）。"""
    flags_sb = getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0) | getattr(cv2, "CALIB_CB_ACCURACY", 0)
    if use_sb and hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(gray, (pattern_cols, pattern_rows), flags_sb)
        if ok:
            return True, corners.astype(np.float32)

    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    ok, corners = cv2.findChessboardCorners(gray, (pattern_cols, pattern_rows), flags)
    if not ok:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def calibrate(image_paths, pattern_cols, pattern_rows, square_mm, verbose=True):
    objp = build_object_points(pattern_cols, pattern_rows, square_mm)
    obj_points, img_points = [], []
    used, failed = [], []

    for p in image_paths:
        img = cv2.imread(p)
        if img is None:
            failed.append((p, "读图失败"))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = find_corners(gray, pattern_cols, pattern_rows)
        if not ok:
            failed.append((p, "找不到棋盘格角点"))
            continue
        obj_points.append(objp.copy())
        img_points.append(corners)
        used.append(p)
        if verbose:
            print(f"  [OK] {os.path.basename(p)}  {img.shape[1]}x{img.shape[0]}  "
                  f"角点数={len(corners)}")

    if len(used) < 3:
        raise SystemExit(
            f"有效标定图只有 {len(used)} 张，至少要 3 张（细则建议 >=15 张）。\n"
            f"失败原因：{failed[:5]}"
        )

    h, w = cv2.imread(used[0]).shape[:2]
    print(f"\n标定中：{len(used)} 张有效图，分辨率 {w}x{h} ...")

    flags = 0
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None, flags=flags)

    # 每张图的重投影误差，挑出最差的几张（现场很好用：说明那张图拍得不好）
    per_view = []
    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, D)
        a = np.asarray(img_points[i], dtype=np.float64).reshape(-1, 2)
        b = np.asarray(proj, dtype=np.float64).reshape(-1, 2)
        err = float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1))))
        per_view.append((err, used[i]))

    per_view.sort(reverse=True)
    return {
        "rms": float(rms),
        "K": K, "D": D.reshape(-1),
        "width": w, "height": h,
        "used": used, "failed": failed,
        "per_view": per_view,
    }


def save_yaml(res, out_path, pattern, square_mm, board_type="chessboard"):
    K, D = res["K"], res["D"]
    data = {
        "camera": {
            "fx": float(K[0, 0]), "fy": float(K[1, 1]),
            "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "dist_coeffs": [float(v) for v in D],
            "image_width": int(res["width"]),
            "image_height": int(res["height"]),
            "rms_reprojection_error": round(res["rms"], 4),
            "calibration_images": len(res["used"]),
            "board_type": board_type,
            "board_size": f"{pattern} 内角点, 方格 {square_mm} mm",
            "calibration_date": __import__("datetime").date.today().isoformat(),
        },
        "K": [[float(v) for v in row] for row in K],
        "D": [float(v) for v in D],
    }
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# 由 scripts/calibrate_camera.py 自动生成\n")
        f.write(f"# 重投影误差 RMS = {res['rms']:.4f} px"
                f"（经验上 < 0.5 可用，< 0.3 很好）\n")
        f.write("# 畸变系数顺序 [k1, k2, p1, p2, k3]\n")
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    return out_path


# ---------------------------------------------------------------- 自检

def make_synthetic_homography(rng, K, objp, img_size):
    """构造一个合法的相机位姿，返回旋转/平移向量。"""
    w, h = img_size
    # 让棋盘格大致落在画面中间，倾斜角适中
    Rx = rng.uniform(-0.55, 0.55)
    Ry = rng.uniform(-0.55, 0.55)
    Rz = rng.uniform(-0.4, 0.4)
    R, _ = cv2.Rodrigues(np.array([Rx, Ry, Rz], dtype=np.float64))
    # 距离：让格子覆盖画面 30%~70%
    span = objp[:, :2].max(axis=0) - objp[:, :2].min(axis=0)
    target_frac = rng.uniform(0.30, 0.70)
    z = max(span) * K[0, 0] / (target_frac * w)
    t = np.array([rng.uniform(-0.15, 0.15) * w * z / K[0, 0],
                  rng.uniform(-0.15, 0.15) * h * z / K[1, 1],
                  z], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(R)
    return rvec, t


def render_checkerboard(objp, K, D, rvec, t, img_size):
    """
    把棋盘格按真实相机模型投影到画面上（含畸变）。

    关键几何关系（很容易搞错）：
        pattern = 9x6 指的是**内角点数**，
        对应的方格数是 (9+1) x (6+1) = 10 x 7 个黑白格，
        才使得内部交点数正好是 9x6 = 54 个。
    如果按 9x6 个方格去画，内角点只有 8x5，findChessboardCorners 必然失败。
    """
    w, h = img_size
    xs = np.unique(np.round(objp[:, 0], 6))
    ys = np.unique(np.round(objp[:, 1], 6))
    sq = xs[1] - xs[0]
    nx, ny = len(xs), len(ys)          # 内角点 9 x 6

    n_sq_x, n_sq_y = nx + 1, ny + 1    # 方格 10 x 7
    img = np.full((h, w), 235, np.uint8)

    # 方格角点：索引用 0..n_sq_x，坐标 (i-1)*sq，这样内角点正好落在 1..nx
    grid = [[cv2.projectPoints(np.array([[(i - 1) * sq, (j - 1) * sq, 0.0]], dtype=np.float64),
                               rvec, t, K, D)[0].reshape(2)
             for j in range(n_sq_y + 1)]
            for i in range(n_sq_x + 1)]

    for i in range(n_sq_x):
        for j in range(n_sq_y):
            if (i + j) % 2 == 0:
                quad = np.array([grid[i][j], grid[i + 1][j],
                                 grid[i + 1][j + 1], grid[i][j + 1]], dtype=np.int32)
                cv2.fillConvexPoly(img, quad, 25)

    uv, _ = cv2.projectPoints(objp, rvec, t, K, D)
    return img, uv.reshape(-1, 2)


def selftest(seed=0, n_views=20, pattern=(9, 6), square_mm=25, verbose=True):
    rng = np.random.default_rng(seed)
    pattern_cols, pattern_rows = pattern

    # 真值：故意选一个有点畸变的镜头，检验能不能反解出 k1/k2
    w, h = 1280, 720
    K_true = np.array([[812.0, 0.0, 642.0],
                       [0.0, 809.0, 357.0],
                       [0.0, 0.0, 1.0]], dtype=np.float64)
    D_true = np.array([-0.145, 0.068, 0.0012, -0.0009, -0.012], dtype=np.float64)

    objp = build_object_points(pattern_cols, pattern_rows, square_mm)

    tmp = os.path.join(ROOT, "_selftest_calib")
    os.makedirs(tmp, exist_ok=True)
    for f in glob.glob(os.path.join(tmp, "*.png")):
        os.remove(f)

    saved = 0
    for i in range(n_views * 3):
        if saved >= n_views:
            break
        rvec, t = make_synthetic_homography(rng, K_true, objp, (w, h))
        img, uv = render_checkerboard(objp, K_true, D_true, rvec, t, (w, h))
        # 角点必须整体在画面内，否则这张图没用
        if np.any(uv[:, 0] < 5) or np.any(uv[:, 0] > w - 5):
            continue
        if np.any(uv[:, 1] < 5) or np.any(uv[:, 1] > h - 5):
            continue
        noise = rng.normal(0, 0.6, img.shape)
        img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        p = os.path.join(tmp, f"synth_{saved:03d}.png")
        cv2.imwrite(p, img)
        saved += 1

    print("=" * 72)
    print(f"标定自检：生成 {saved} 张合成棋盘格（{pattern_cols}x{pattern_rows} 内角点，"
          f"方格 {square_mm}mm）")
    print("=" * 72)
    print("真值内参：")
    print(f"  fx={K_true[0,0]:.3f}  fy={K_true[1,1]:.3f}  "
          f"cx={K_true[0,2]:.3f}  cy={K_true[1,2]:.3f}")
    print(f"  dist(k1,k2,p1,p2,k3)={np.round(D_true, 6).tolist()}")

    paths = sorted(glob.glob(os.path.join(tmp, "*.png")))
    res = calibrate(paths, pattern_cols, pattern_rows, square_mm, verbose=verbose)

    K, D = res["K"], res["D"]
    print("\n" + "=" * 72)
    print("反解结果 vs 真值")
    print("=" * 72)
    rows = [
        ("fx", K[0, 0], K_true[0, 0]),
        ("fy", K[1, 1], K_true[1, 1]),
        ("cx", K[0, 2], K_true[0, 2]),
        ("cy", K[1, 2], K_true[1, 2]),
    ]
    ok = True
    for name, est, true in rows:
        rel = abs(est - true) / abs(true) * 100.0
        flag = "OK " if rel < 0.5 else ("可接受" if rel < 2.0 else "偏差大")
        if rel >= 2.0:
            ok = False
        print(f"  {name}: 估计={est:9.3f}  真值={true:9.3f}  "
              f"相对误差={rel:6.3f}%   {flag}")
    print(f"\n  重投影误差 RMS = {res['rms']:.4f} px  "
          f"（真值模型下应该很小）")

    print("\n  畸变系数：")
    names = ["k1", "k2", "p1", "p2", "k3"]
    for i, nm in enumerate(names):
        est, true = float(D[i]), float(D_true[i])
        print(f"    {nm}: 估计={est:+.6f}  真值={true:+.6f}")

    save_yaml(res, os.path.join(ROOT, "calibration", "camera_intrinsics_selftest.yaml"),
              f"{pattern_cols}x{pattern_rows}", square_mm)
    print(f"\n  结果已写入 calibration/camera_intrinsics_selftest.yaml")

    print("\n" + "=" * 72)
    if ok and res["rms"] < 1.0:
        print("自检通过：标定流程能在无相机情况下正确反解内参。")
        print("相机到货后直接拍真实棋盘格即可复用同一套代码。")
    else:
        print("自检偏差偏大，需检查合成图质量或角点检测。")
    print("=" * 72)
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="单目相机内参标定")
    ap.add_argument("--images", help="标定图目录或通配符")
    ap.add_argument("--pattern", default="9x6", help="内角点数，例如 9x6")
    ap.add_argument("--square", type=float, default=25.0, help="方格边长，毫米")
    ap.add_argument("--out", default=os.path.join(ROOT, "calibration", "camera_intrinsics.yaml"))
    ap.add_argument("--selftest", action="store_true", help="不用相机，用合成图自检")
    ap.add_argument("--views", type=int, default=20, help="自检生成多少张")
    args = ap.parse_args()

    if args.selftest:
        cols, rows = (int(v) for v in args.pattern.lower().split("x"))
        return selftest(n_views=args.views, pattern=(cols, rows),
                        square_mm=args.square)

    if not args.images:
        ap.error("需要 --images 或 --selftest")

    if os.path.isdir(args.images):
        paths = sorted(glob.glob(os.path.join(args.images, "*.*")))
        paths = [p for p in paths if p.lower().endswith((".png", ".jpg", ".jpeg", ".bmp"))]
    else:
        paths = sorted(glob.glob(args.images))
    if not paths:
        raise SystemExit(f"没找到图片: {args.images}")

    cols, rows = (int(v) for v in args.pattern.lower().split("x"))
    print(f"找到 {len(paths)} 张候选图，开始找角点 ...")
    res = calibrate(paths, cols, rows, args.square)

    print("\n" + "=" * 70)
    print(f"重投影误差 RMS = {res['rms']:.4f} px")
    print(f"  fx={res['K'][0,0]:.3f}  fy={res['K'][1,1]:.3f}  "
          f"cx={res['K'][0,2]:.3f}  cy={res['K'][1,2]:.3f}")
    print(f"  畸变 = {np.round(res['D'], 6).tolist()}")
    if res["rms"] > 0.5:
        print("\n  ⚠️ RMS 偏大，通常是这几张图的问题（误差从大到小）：")
        for err, p in res["per_view"][:3]:
            print(f"     {err:.4f} px  {os.path.basename(p)}")
        print("  建议删掉最差的 1~2 张重标，或补拍四角/倾斜角度更大的图。")
    if res["failed"]:
        print(f"\n  有 {len(res['failed'])} 张图没找到角点：")
        for p, why in res["failed"][:5]:
            print(f"     {os.path.basename(p)}: {why}")

    out = save_yaml(res, args.out, f"{cols}x{rows}", args.square)
    print(f"\n已写入 {out}")
    print("注意：标定分辨率必须与推理分辨率一致；改分辨率要重新标定。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
