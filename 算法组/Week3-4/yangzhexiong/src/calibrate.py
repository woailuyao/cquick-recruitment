"""相机内参标定：棋盘格 → camera_intrinsics.yaml

用法：
    python3 calibrate.py                       # 读 calib/*.jpg，默认 9x6 内角点、20mm 格
    python3 calibrate.py --cols 9 --rows 6 --square 20
    python3 calibrate.py --dir calib --out calibration/camera_intrinsics.yaml

拍图要求：
    · 至少 15 张，最好 20 张
    · 棋盘占画面 1/3 到 2/3
    · 覆盖中心、四角、不同倾斜角
    · 棋盘必须平整、图必须清晰

判定标准（脚本会打印）：
    重投影误差 RMS < 0.5 px → 很好
    0.5 ~ 1.0 px           → 可用
    > 1.0 px               → 建议删掉拍歪的图重标
"""
import argparse
import glob
import os

import cv2
import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="calib")
    ap.add_argument("--cols", type=int, default=9, help="内角点列数（宽）")
    ap.add_argument("--rows", type=int, default=6, help="内角点行数（高）")
    ap.add_argument("--square", type=float, default=20.0, help="每格实际边长 mm")
    ap.add_argument("--out", default="calibration/camera_intrinsics.yaml")
    ap.add_argument("--fix-k3", action="store_true",
                    help="固定 k3=0（5 参数模型），k2/k3 过大时用")
    ap.add_argument("--fix-k2-k3", action="store_true",
                    help="固定 k2=k3=0（只用 k1 的简单模型），畸变极不稳定时用")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    # 三维棋盘点（Z=0 平面）
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    objp *= args.square

    objpoints, imgpoints = [], []
    files = sorted(glob.glob(os.path.join(args.dir, "*.jpg")) +
                   glob.glob(os.path.join(args.dir, "*.png")))
    if not files:
        print(f"{args.dir} 下没有图片，请先拍照")
        return 1

    size = None
    used = 0
    for path in files:
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(
            gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            print(f"  [x] 没找到角点：{os.path.basename(path)}")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        used += 1
        print(f"  [v] {os.path.basename(path)}  找到 {len(corners)} 个角点")

    print(f"\n成功使用 {used}/{len(files)} 张")
    if used < 10:
        print("少于 10 张，标定不可靠，请补拍")
        return 1

    flags = 0
    if args.fix_k3:
        flags |= cv2.CALIB_FIX_K3
    if args.fix_k2_k3:
        flags |= cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, size, None, None, flags=flags)
    print(f"\n重投影误差 RMS = {rms:.4f} px")
    if rms < 0.5:
        print("  评价：很好")
    elif rms < 1.0:
        print("  评价：可用")
    else:
        print("  评价：偏大，建议删掉拍歪/模糊的图重标")

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    k1, k2, p1, p2, k3 = dist.ravel()[:5]
    print(f"\nfx={fx:.2f}  fy={fy:.2f}  cx={cx:.2f}  cy={cy:.2f}")
    print(f"k1={k1:.6f} k2={k2:.6f} p1={p1:.6f} p2={p2:.6f} k3={k3:.6f}")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    data = {
        "image_width": int(size[0]),
        "image_height": int(size[1]),
        "reprojection_error_px": float(rms),
        "calib_images_used": used,
        "camera_matrix": {
            "fx": float(fx), "fy": float(fy), "cx": float(cx), "cy": float(cy),
        },
        "dist_coeffs": {
            "k1": float(k1), "k2": float(k2), "p1": float(p1), "p2": float(p2), "k3": float(k3),
        },
        "note": "dist order: k1 k2 p1 p2 k3",
    }
    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"\n已保存：{args.out}")

    # 顺手存一张去畸变对比图，方便检查
    test = cv2.imread(files[0])
    if test is not None:
        h, w = test.shape[:2]
        newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
        fixed = cv2.undistort(test, K, dist, None, newK)
        os.makedirs("outputs", exist_ok=True)
        cv2.imwrite("outputs/undistort_before.jpg", test)
        cv2.imwrite("outputs/undistort_after.jpg", fixed)
        print("去畸变对比图：outputs/undistort_before.jpg / undistort_after.jpg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

