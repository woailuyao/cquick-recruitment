# -*- coding: utf-8 -*-
"""相机内参标定（棋盘格，带质量诊断与模型自选）

为什么要重写：
    上一次标定得到 k3 = -14.71、cx = 589.6、cy = 323.1，这组值非常可疑。
    常见原因是：
      1) 用的不是 1280x720 原图（截图 / 播放器缩放 / 裁剪过窗口），
         标出来的内参只对"那张截图的分辨率"成立，用到 720p 上必然整体偏大或偏小；
      2) 拍板子时姿态太单一（都同一个距离、同一个倾角），
         高阶畸变系数（k3）被"拟合"出来，实际是过拟合；
      3) 板子没铺满画面，四角没有角点，径向畸变根本没被约束住。
    这个脚本把这三件事全部做成硬检查：分辨率必须一致且等于期望值、按姿态分层报告、
    自动在"3/4/5 参数"里选模型、并做畸变模型自检（反畸变后不能折叠）。

用法：
    python calibrate_camera.py --images images/calib
    python calibrate_camera.py --images images/calib --expect 1280x720 --min-images 15
    python calibrate_camera.py --images images/calib --auto-pattern   # 自动试几种格数

输出：
    camera_intrinsics.yaml      可直接替换 config/camera.yaml 里的内参段
    calib_debug/                每张图的角点叠加图 + 去畸变前后对比 + 误差表
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import cv2
import numpy as np

# ---------------------------------------------------------------- 参数 ----
DEFAULT_COLS, DEFAULT_ROWS = 9, 9      # 内角点数 = 方格数-1；10x10 方格 -> 9x9
DEFAULT_SQUARE = 0.025                 # 每格边长（米）；内参标定其实不需要真实尺寸
DEFAULT_EXPECT = "1280x720"


# ------------------------------------------------------------ 角点检测 ----
def detect_corners(gray, pattern, debug=False):
    """优先 SB 算法（抗噪、不需要二值化阈值），失败回退经典算法 + 亚像素。"""
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        if debug:
            flags |= cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags)
        if ok:
            return True, corners.astype(np.float32), "SB"

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return False, None, "none"
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-4)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners, "classic"


def board_pose_features(corners, shape, cols, rows):
    """提取这张图的"姿态特征"，用来诊断是否姿态太单一。

    返回 (中心归一化坐标 x, 中心归一化坐标 y, 板子外接框面积占比, 上下边宽度比)
    """
    h, w = shape
    pts = corners.reshape(-1, 2)
    cx = (pts[:, 0].mean()) / w
    cy = (pts[:, 1].mean()) / h
    bw = pts[:, 0].max() - pts[:, 0].min()
    bh = pts[:, 1].max() - pts[:, 1].min()
    area_ratio = (bw * bh) / float(w * h)
    tilt = None
    if len(pts) == cols * rows:
        grid = pts.reshape((rows, cols, 2))
        top = np.linalg.norm(grid[0, -1] - grid[0, 0])
        bot = np.linalg.norm(grid[-1, -1] - grid[-1, 0])
        if max(top, bot) > 1e-6:
            tilt = float(min(top, bot) / max(top, bot))  # 越接近 0 越斜
    return cx, cy, area_ratio, tilt


# ---------------------------------------------------- 畸变模型自检 ----
def undistort_sanity(K, dist, width, height):
    """检查畸变模型是否"物理上成立"：反畸变后像素坐标必须保持单调，不能折叠。

    折叠 = 图像边缘被拉回去重叠，说明高阶系数（通常是 k3）已经发散。
    返回 (是否通过, 最大位移px, 说明)
    """
    us = np.linspace(0, width - 1, 40)
    vs = np.linspace(0, height - 1, 24)
    grid = np.array([[u, v] for v in vs for u in us], np.float32).reshape(-1, 1, 2)
    und = cv2.undistortPoints(grid, K, dist, P=K).reshape(len(vs), len(us), 2)

    # 每一行的 x 必须递增，每一列的 y 必须递增
    dx = np.diff(und[:, :, 0], axis=1)
    dy = np.diff(und[:, :, 1], axis=0)
    folded = bool((dx <= 0).any() or (dy <= 0).any())
    disp = np.linalg.norm(und.reshape(-1, 2) - grid.reshape(-1, 2), axis=1).max()
    msg = "反畸变映射单调，模型可用"
    if folded:
        msg = "反畸变映射出现折叠！高阶畸变系数不合理，必须降低模型阶数或重拍标定图"
    return (not folded), float(disp), msg


def calibrate_once(objpoints, imgpoints, size, flags):
    K0 = np.eye(3, dtype=np.float64)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9)
    return cv2.calibrateCamera(objpoints, imgpoints, size, K0, None,
                               flags=flags, criteria=criteria)


def per_image_errors(objpoints, imgpoints, rvecs, tvecs, K, dist):
    errs = []
    for op, ip, rv, tv in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv2.projectPoints(op, rv, tv, K, dist)
        e = np.linalg.norm(ip.reshape(-1, 2) - proj.reshape(-1, 2), axis=1)
        errs.append(float(e.mean()))
    return np.array(errs)


# ------------------------------------------------------------------ 主流程 ----
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="棋盘格图片目录")
    ap.add_argument("--cols", type=int, default=DEFAULT_COLS)
    ap.add_argument("--rows", type=int, default=DEFAULT_ROWS)
    ap.add_argument("--square", type=float, default=DEFAULT_SQUARE, help="每格边长(米)")
    ap.add_argument("--expect", default=DEFAULT_EXPECT, help="期望分辨率，如 1280x720")
    ap.add_argument("--min-images", type=int, default=15)
    ap.add_argument("--auto-pattern", action="store_true", help="自动尝试几种内角点数")
    ap.add_argument("--out-yaml", default="camera_intrinsics.yaml")
    ap.add_argument("--debug-dir", default="calib_debug")
    args = ap.parse_args()

    exp_w, exp_h = (int(x) for x in args.expect.lower().split("x"))

    files = []
    for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
        files.extend(glob.glob(os.path.join(args.images, ext)))
    files = sorted(files)
    if not files:
        sys.exit(f"[ERROR] {args.images} 下没有图片")

    # ---------- 1. 读图 + 分辨率硬检查 ----------
    images, grays, sizes, bad_size = [], [], set(), []
    for f in files:
        img = cv2.imread(f)
        if img is None:
            print(f"[SKIP] 读不出 {f}")
            continue
        h, w = img.shape[:2]
        sizes.add((w, h))
        if (w, h) != (exp_w, exp_h):
            bad_size.append((os.path.basename(f), w, h))
        images.append((f, img))
        grays.append(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))

    print(f"[INFO] 读到 {len(images)} 张图，分辨率集合 = {sizes}")
    if len(sizes) > 1:
        print("[FATAL] 分辨率不一致，无法标定。混进来的尺寸如下：")
        for n, w, h in bad_size:
            print(f"        {n}: {w}x{h}")
        sys.exit(1)
    if bad_size:
        print(f"[FATAL] 所有图都是 {list(sizes)[0]}，不是期望的 {exp_w}x{exp_h}。")
        print("        这几乎肯定是截图/播放器缩放造成的 —— 内参会整体缩放，")
        print("        用到 720p 视频上会完全对不上。请用下面的方式重新取图：")
        print("            python frames.py D:\\视频目录 D:\\calib_frames")
        print("        或者用 cv2.VideoCapture 直接 read() 保存，不要用截图工具。")
        sys.exit(1)

    # ---------- 2. 选棋盘格内角点数 ----------
    patterns = [(args.cols, args.rows)]
    if args.auto_pattern:
        patterns += [(8, 8), (10, 10), (9, 6), (7, 5), (11, 11), (6, 9)]
        patterns = list(dict.fromkeys(patterns))

    best_pattern, best_hits, best_data = None, -1, None
    for pat in patterns:
        ok_list, corners_list = [], []
        for f, g in zip([i[0] for i in images], grays):
            ok, corners, how = detect_corners(g, pat)
            ok_list.append(ok)
            corners_list.append(corners)
        hits = sum(ok_list)
        print(f"[INFO] 内角点 {pat[0]}x{pat[1]} -> 检出 {hits}/{len(images)} 张")
        if hits > best_hits:
            best_pattern, best_hits, best_data = pat, hits, (ok_list, corners_list)
        if hits >= max(args.min_images, int(0.8 * len(images))):
            break

    if best_hits < 5:
        sys.exit("[ERROR] 有效图片太少，检查棋盘格大小写、光照、是否整块入镜")

    cols, rows = best_pattern
    ok_list, corners_list = best_data
    print(f"[INFO] 采用内角点 {cols}x{rows}，有效 {best_hits} 张")

    # ---------- 3. 组装 3D-2D 对应 ----------
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= args.square

    objpoints, imgpoints, used_files, used_corners, feats = [], [], [], [], []
    for (f, img), g, ok, c in zip(images, grays, ok_list, corners_list):
        if not ok:
            continue
        objpoints.append(objp.copy())
        imgpoints.append(c.astype(np.float32))
        used_files.append(f)
        used_corners.append(c)
        feats.append(board_pose_features(c, (exp_h, exp_w), cols, rows))

    if len(used_files) < args.min_images:
        print(f"[WARN] 有效图只有 {len(used_files)} 张（建议 >= {args.min_images}），"
              f"标定结果可能不稳")

    # ---------- 4. 姿态多样性诊断 ----------
    cx_s = np.array([f[0] for f in feats])
    cy_s = np.array([f[1] for f in feats])
    area = np.array([f[2] for f in feats])
    print("\n--- 姿态多样性诊断 ---")
    print(f"  板中心归一化范围: x {cx_s.min():.2f}~{cx_s.max():.2f}   "
          f"y {cy_s.min():.2f}~{cy_s.max():.2f}")
    print(f"  板子在画面中的占比: {area.min()*100:.0f}%~{area.max()*100:.0f}%")
    if area.mean() < 0.20:
        print("  [WARN] 板子整体偏小，径向畸变约束不足 -> k3 容易乱跑")
    if np.ptp(cx_s) < 0.15 or np.ptp(cy_s) < 0.15:
        print("  [WARN] 板子位置变化太少（一直待在画面同一块），四角没被覆盖")

    # ---------- 5. 三种模型各标一次，自动选 ----------
    models = [
        ("5参数 k1,k2,p1,p2,k3", 0, 5),
        ("4参数 k1,k2,p1,p2 (k3=0)", cv2.CALIB_FIX_K3, 4),
        ("3参数 k1,k2 (切向=0,k3=0)", cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3, 3),
    ]
    print("\n--- 模型对比 ---")
    results = []
    for name, flags, npar in models:
        try:
            rms, K, dist, rv, tv = calibrate_once(objpoints, imgpoints, (exp_w, exp_h), flags)
        except cv2.error as e:
            print(f"  {name}: 失败 ({e})")
            continue
        ok, disp, msg = undistort_sanity(K, dist, exp_w, exp_h)
        d = dist.ravel()
        k3 = d[4] if len(d) > 4 else 0.0
        print(f"  {name:28s} RMS={rms:7.4f} px  k1={d[0]:+.3f} k2={d[1]:+.3f} "
              f"k3={k3:+.3f}  最大反畸变位移={disp:6.1f}px  {'OK' if ok else '折叠!'}")
        results.append({"name": name, "flags": flags, "npar": npar, "rms": rms,
                        "K": K, "dist": dist, "rvecs": rv, "tvecs": tv,
                        "sane": ok, "disp": disp})

    if not results:
        sys.exit("[ERROR] 三种模型都标定失败")

    # 选型规则：精度接近时优先参数少的；畸变模型必须通过自检
    sane = [r for r in results if r["sane"]]
    pool = sane if sane else results
    best_rms = min(r["rms"] for r in pool)
    tol = 0.05  # px，5 参数比 4 参数好不到 0.05px 就不值得用它
    chosen = min([r for r in pool if r["rms"] <= best_rms + tol],
                 key=lambda r: (r["npar"], r["rms"]))
    print(f"\n[CHOOSE] {chosen['name']}  (RMS={chosen['rms']:.4f} px)")
    if not chosen["sane"]:
        print("[WARN] 选中的模型也没通过折叠自检，强烈建议重拍标定图")

    K, dist = chosen["K"], chosen["dist"]
    rms = chosen["rms"]

    # ---------- 6. 剔除最差的图再标一次 ----------
    errs = per_image_errors(objpoints, imgpoints, chosen["rvecs"], chosen["tvecs"], K, dist)
    order = np.argsort(errs)[::-1]
    print("\n--- 每张图重投影误差（差的在前）---")
    for i in order[:6]:
        print(f"  {errs[i]:7.4f} px  {os.path.basename(used_files[i])}")
    keep = [i for i in range(len(used_files)) if errs[i] <= max(1.5, errs.mean() * 2.0)]
    if len(keep) >= max(10, args.min_images) and len(keep) < len(used_files):
        print(f"\n[INFO] 剔除 {len(used_files) - len(keep)} 张离群图后重标 "
              f"({len(keep)} 张)")
        obj2 = [objpoints[i] for i in keep]
        img2 = [imgpoints[i] for i in keep]
        rms2, K2, dist2, rv2, tv2 = calibrate_once(obj2, img2, (exp_w, exp_h), chosen["flags"])
        ok2, _, _ = undistort_sanity(K2, dist2, exp_w, exp_h)
        if rms2 < rms and ok2:
            print(f"       RMS {rms:.4f} -> {rms2:.4f} px，采用剔除后的结果")
            rms, K, dist, rv, tv = rms2, K2, dist2, rv2, tv2
            objpoints, imgpoints, used_files, used_corners = obj2, img2, \
                [used_files[i] for i in keep], [used_corners[i] for i in keep]
            errs = per_image_errors(objpoints, imgpoints, rv, tv, K, dist)

    # ---------- 7. 最终报告 ----------
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    d = np.zeros(5)
    d[:len(dist.ravel())] = dist.ravel()
    hfov = 2 * np.degrees(np.arctan(exp_w / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(exp_h / (2 * fy)))
    ok, disp, msg = undistort_sanity(K, dist, exp_w, exp_h)

    print("\n================ 最终结果 ================")
    print(f"  图像尺寸      : {exp_w}x{exp_h}")
    print(f"  RMS           : {rms:.4f} px   {'[可用]' if rms < 1.0 else '[偏大，建议重标]'}")
    print(f"  fx, fy        : {fx:.3f}, {fy:.3f}   (比值 {fx/fy:.4f})")
    print(f"  cx, cy        : {cx:.3f}, {cy:.3f}   "
          f"(相对中心偏移 {cx-exp_w/2:+.1f}, {cy-exp_h/2:+.1f} px)")
    print(f"  dist k1,k2,p1,p2,k3 : {d[0]:+.5f}, {d[1]:+.5f}, {d[2]:+.5f}, "
          f"{d[3]:+.5f}, {d[4]:+.5f}")
    print(f"  视场角        : 水平 {hfov:.1f}°  垂直 {vfov:.1f}°")
    print(f"  畸变自检      : {'通过' if ok else '不通过'}（最大反畸变位移 {disp:.1f} px）{msg}")
    print(f"  使用模型      : {chosen['name']}")
    print("==========================================")

    # ---------- 8. 存 YAML（可直接替换 config/camera.yaml 的内参段） ----------
    yaml_text = f"""# 相机内参标定结果（calibrate_camera.py 自动生成）
# 图像尺寸: {exp_w}x{exp_h}
# 棋盘格内角点: {cols}x{rows}，方格边长 {args.square} m
# 使用模型: {chosen['name']}
# 标定图数量: {len(used_files)}
image_width: {exp_w}
image_height: {exp_h}

camera_matrix:
  fx: {fx:.6f}
  fy: {fy:.6f}
  cx: {cx:.6f}
  cy: {cy:.6f}

# OpenCV 顺序: [k1, k2, p1, p2, k3]
dist_coeffs: [{d[0]:.6f}, {d[1]:.6f}, {d[2]:.6f}, {d[3]:.6f}, {d[4]:.6f}]

rms_reprojection_error: {rms:.6f}
fov_horizontal_deg: {hfov:.3f}
fov_vertical_deg: {vfov:.3f}
K_3x3: [{fx:.6f}, 0, {cx:.6f}, 0, {fy:.6f}, {cy:.6f}, 0, 0, 1]

# 安装外参（用卷尺/角度尺量，或按你的"合页 120°"几何算）
mount:
  height_m: 0.98        # 相机光心离地高度
  pitch_deg: 30.0       # 俯角，向下为正
  roll_deg: 0.0
  yaw_deg: 0.0
"""
    with open(args.out_yaml, "w", encoding="utf-8") as f:
        f.write(yaml_text)
    print(f"[SAVE] {args.out_yaml}")

    # ---------- 9. 调试图 ----------
    os.makedirs(args.debug_dir, exist_ok=True)
    for f, c, e in zip(used_files, used_corners, errs):
        img = cv2.imread(f)
        cv2.drawChessboardCorners(img, (cols, rows), c, True)
        cv2.putText(img, f"err={e:.3f}px", (15, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 0, 255), 2)
        cv2.imwrite(os.path.join(args.debug_dir, "det_" + os.path.basename(f)), img)

    sample = cv2.imread(used_files[0])
    newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (exp_w, exp_h), 1, (exp_w, exp_h))
    und = cv2.undistort(sample, K, dist, None, newK)
    cv2.imwrite(os.path.join(args.debug_dir, "undistort_before.png"), sample)
    cv2.imwrite(os.path.join(args.debug_dir, "undistort_after.png"), und)

    with open(os.path.join(args.debug_dir, "error_table.txt"), "w", encoding="utf-8") as f:
        for name, e in sorted(zip([os.path.basename(x) for x in used_files], errs),
                              key=lambda t: -t[1]):
            f.write(f"{e:.4f}\t{name}\n")
        f.write(f"\nRMS={rms:.4f} px  model={chosen['name']}\n")
    print(f"[SAVE] {args.debug_dir}/ (角点图、去畸变对比、误差表)")

    print("\n下一步：把 camera_intrinsics.yaml 里的 fx/fy/cx/cy/dist_coeffs 抄进 "
          "config/camera.yaml，然后跑 selftest_geometry.py 看视场范围是否合理。")


if __name__ == "__main__":
    main()
