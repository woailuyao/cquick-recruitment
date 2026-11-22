"""
相机内参与畸变标定求解（任务 2 · 2.5 节）
=================================================
输入：config/calib_board.yaml 指定的标定板参数 + D:/W34_dataset/calib 下的标定照片
输出：config/camera_intrinsics.yaml
      - camera_matrix: 3x3 内参矩阵 [[fx,0,cx],[0,fy,cy],[0,0,1]]
      - dist_coeffs:   畸变系数，OpenCV 顺序 [k1, k2, p1, p2, k3]
      - image_width / image_height: 标定与推理统一分辨率 1280x720
      - reprojection_error: 全局重投影误差（px，越小越好，一般 <0.3）

用法（Python 3.10 全路径）：
  C:\\Users\\A\\AppData\\Local\\Programs\\Python\\Python310\\python.exe calibrate_intrinsics.py
"""
import os
import cv2
import numpy as np
import yaml

# ===== 路径 =====
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD_CFG = os.path.join(BASE_DIR, "config", "calib_board.yaml")
OUT_YAML = os.path.join(BASE_DIR, "config", "camera_intrinsics.yaml")


def load_board_cfg():
    """读取标定板配置：内角点数量、格边长(米)、分辨率、图片目录、剔除清单。"""
    with open(BOARD_CFG, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    return cfg


def main():
    cfg = load_board_cfg()

    cols = int(cfg["pattern_cols"])        # 横向内角点 7
    rows = int(cfg["pattern_rows"])       # 纵向内角点 5
    square_m = float(cfg["square_size_m"])  # 实测格边长，米（28mm = 0.028）
    img_w = int(cfg["image_width"])
    img_h = int(cfg["image_height"])
    img_dir = cfg["image_dir"]
    reject = set(cfg.get("reject_images", []))

    # 角点子像素精化的终止条件：最多迭代 30 次，或角点位移小于 1e-3
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    # ---- 世界坐标棋盘格角点（Z=0 平面）----
    # 生成 (0,0),(1,0),...,(cols-1,rows-1) 的格点编号，再乘格边长得到米制坐标
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m

    objpoints = []   # 每张图对应的世界坐标点集
    imgpoints = []    # 每张图对应的图像坐标点集
    used, failed = [], []

    files = sorted(f for f in os.listdir(img_dir)
                   if f.lower().endswith(".jpg") and f not in reject)

    for fn in files:
        path = os.path.join(img_dir, fn)
        img = cv2.imread(path)
        if img is None:
            failed.append((fn, "读不到"))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 检测 7x5 内角点
        ok, corners = cv2.findChessboardCorners(
            gray, (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok or corners is None:
            failed.append((fn, "角点 0"))
            continue

        # 子像素精化，提升角点定位精度
        corners_ref = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners_ref)
        used.append(fn)

    print(f"有效图 {len(used)} 张；失败 {len(failed)} 张")
    for fn, why in failed:
        print(f"  [剔除] {fn}: {why}")

    if len(objpoints) < 5:
        raise SystemExit("有效标定图不足 5 张，无法求解内参，请先补拍。")

    # ---- 求解内参矩阵与畸变系数 ----
    # camera_matrix / dist_coeffs / rvecs / tvecs / 全局重投影误差 rms
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (img_w, img_h), None, None)

    # ---- 逐图重投影误差，便于定位坏图 ----
    per_image_err = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        # 两边都压成 (N,2) 并转同一 dtype，避免 OpenCV norm 类型不一致报错
        a = imgpoints[i].reshape(-1, 2).astype(np.float64)
        b = proj.reshape(-1, 2).astype(np.float64)
        err = cv2.norm(a, b, cv2.NORM_L2) / len(proj)
        per_image_err.append((used[i], round(float(err), 4)))

    mean_err = float(np.mean([e for _, e in per_image_err]))

    # ---- 整理输出（numpy 类型转 Python 原生 float，便于 YAML 序列化）----
    K = K.astype(float)
    dist = dist.astype(float).flatten()
    intrinsics = {
        "map_version": "C0-v1",
        "calib_date": cfg.get("measured_date", ""),
        "image_width": img_w,
        "image_height": img_h,
        "camera_matrix": {
            "fx": round(float(K[0, 0]), 6),
            "fy": round(float(K[1, 1]), 6),
            "cx": round(float(K[0, 2]), 6),
            "cy": round(float(K[1, 2]), 6),
            "matrix": [[round(float(K[0, 0]), 6), 0.0, round(float(K[0, 2]), 6)],
                       [0.0, round(float(K[1, 1]), 6), round(float(K[1, 2]), 6)],
                       [0.0, 0.0, 1.0]],
        },
        # OpenCV 顺序：[k1, k2, p1, p2, k3]
        "dist_coeffs": {
            "order": ["k1", "k2", "p1", "p2", "k3"],
            "k1": round(float(dist[0]), 6),
            "k2": round(float(dist[1]), 6),
            "p1": round(float(dist[2]), 6),
            "p2": round(float(dist[3]), 6),
            "k3": round(float(dist[4]), 6),
            "coeffs": [round(float(dist[j]), 6) for j in range(5)],
        },
        "reprojection_error": {
            "rms": round(float(rms), 4),
            "mean_per_image": round(mean_err, 4),
            "per_image": [{"image": n, "error": e} for n, e in per_image_err],
        },
        "used_images": used,
        "rejected_images": [fn for fn, _ in failed],
        "note": "标定分辨率 1280x720；改变分辨率/裁剪/缩放必须重新标定或同步调整内参。"
                "首版棋盘样本多落在画面中下部、上下覆盖不足，主点 cy 与 k3 偏大；"
                "考核前补拍覆盖画面上部/四角/不同距离的图后复标。",
    }

    with open(OUT_YAML, "w", encoding="utf-8") as f:
        yaml.safe_dump(intrinsics, f, allow_unicode=True, sort_keys=False, width=120)

    print("\n=== 标定结果 ===")
    print(f"fx={K[0,0]:.3f} fy={K[1,1]:.3f} cx={K[0,2]:.3f} cy={K[1,2]:.3f}")
    print(f"dist=[{', '.join(f'{d:.5f}' for d in dist)}]")
    print(f"全局 RMS 重投影误差={rms:.4f} px；逐图平均={mean_err:.4f} px")
    print(f"已写出: {OUT_YAML}")


if __name__ == "__main__":
    main()
