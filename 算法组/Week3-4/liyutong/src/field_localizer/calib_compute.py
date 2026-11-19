# -*- coding: utf-8 -*-
"""
内参计算脚本：跑完 calib_capture.py 采集 >=15 张后运行
用法：conda activate calib && python calib_compute.py
输出：相机矩阵 K、畸变系数、重投影误差，保存 calibration.npz 和校验图
考核要求：必须保留 fx/fy/cx/cy 和畸变系数，重投影误差越小越好（通常 < 0.5 px 合格）
"""
import cv2
import glob
import os
import numpy as np

BOARD = (9, 6)          # 与 calib_capture.py 保持一致
SQUARE_MM = 20.0        # 每个格子的实际边长（毫米），用尺子量你打印的棋盘后改这里
IMG_DIR = "calib_imgs"

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * SQUARE_MM

def imread_safe(path):
    """cv2.imread 遇到中文路径返回 None，用 imdecode 绕过"""
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR) if data.size else None

def imwrite_safe(path, img):
    ok, buf = cv2.imencode(os.path.splitext(path)[1], img)
    if ok:
        buf.tofile(path)
    return ok

objpoints, imgpoints = [], []
imgs = sorted(glob.glob(os.path.join(IMG_DIR, "*.jpg")))
print(f"找到 {len(imgs)} 张标定图")
if len(imgs) < 15:
    print("警告：不足 15 张，考核会扣分，建议回 calib_capture.py 补拍")

used = 0
for path in imgs:
    img = imread_safe(path)
    if img is None:
        print(f"读不出来（路径或编码问题）：{os.path.basename(path)}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, BOARD, None)
    if not ok:
        print(f"跳过（未检出角点）：{os.path.basename(path)}")
        continue
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    objpoints.append(objp)
    imgpoints.append(corners)
    used += 1

print(f"有效标定图：{used} 张")
if used < 3:
    print("有效图太少，重新采集")
    raise SystemExit

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("=" * 50)
print(f"重投影误差 RMS: {ret:.4f} px   (<0.5 合格, >1.0 建议重拍)")
print("相机矩阵 K:")
print(K)
print("畸变系数 (k1,k2,p1,p2,k3):")
print(dist.ravel())
print("=" * 50)

np.savez("calibration.npz", K=K, dist=dist)
print("已保存 calibration.npz —— 任务 2 地面投影要用的就是它")

# 同时输出考核提交要求的内参 yaml（calibration/camera_intrinsics.yaml 的格式）
K_list = ", ".join(f"{v:.6f}" for v in K.ravel())
d_list = ", ".join(f"{v:.6f}" for v in dist.ravel())
with open("camera_intrinsics.yaml", "w", encoding="utf-8") as f:
    f.write(f"""# 由 calib_compute.py 自动生成，calibrated_date / num_images 照实填
calibrated_date: ""
num_images: {used}
board: "{BOARD[0]}x{BOARD[1]}"
square_mm: {SQUARE_MM}

image_width: {gray.shape[1]}
image_height: {gray.shape[0]}

camera_matrix:
  rows: 3
  cols: 3
  data: [{K_list}]

distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{d_list}]

reprojection_error_px: {ret:.4f}
""")
print("已保存 camera_intrinsics.yaml（直接拷进提交目录 calibration/）")

# 生成一张去畸变前后对比图，评审可能要看
sample = imread_safe(imgs[0])
h, w = sample.shape[:2]
new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1)
undist = cv2.undistort(sample, K, dist, None, new_K)
compare = np.hstack([sample, undist])
imwrite_safe("undistort_check.jpg", compare)
print("已生成对比图 undistort_check.jpg（左原图右去畸变）")
