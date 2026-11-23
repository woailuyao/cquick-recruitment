# -*- coding: utf-8 -*-
"""标定体检：逐张重投影误差 + 倾斜多样性 + 剔除坏样本后重算"""
import cv2, glob, numpy as np, os

BOARD = (9, 6)
SQUARE_MM = 20.0
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * SQUARE_MM

def imread_safe(p):
    d = np.fromfile(p, np.uint8); return cv2.imdecode(d, cv2.IMREAD_COLOR) if d.size else None

files = sorted(glob.glob("calib_imgs/*.jpg"))
obj_all, img_all, ok_files, shapes = [], [], [], []
for f in files:
    img = imread_safe(f); gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, c = cv2.findChessboardCorners(gray, BOARD, None)
    if not ok: continue
    c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), criteria)
    obj_all.append(objp); img_all.append(c); ok_files.append(os.path.basename(f)); shapes.append(gray.shape)

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_all, img_all, shapes[0][::-1], None, None)
print(f"全部 {len(ok_files)} 张: RMS {ret:.4f} px\n")

rows = []
for i, f in enumerate(ok_files):
    proj, _ = cv2.projectPoints(obj_all[i], rvecs[i], tvecs[i], K, dist)
    err = np.linalg.norm(img_all[i].reshape(-1, 2) - proj.reshape(-1, 2), axis=1).mean()
    R, _ = cv2.Rodrigues(rvecs[i])
    tilt = np.degrees(np.arccos(abs(R[2, 2])))          # 板面法线与光轴夹角
    board_px = cv2.contourArea(img_all[i])
    rows.append((err, f, tilt, board_px, np.linalg.norm(tvecs[i])))

print(f"{'文件':28s}{'误差px':>8s}{'倾角°':>7s}{'棋盘面积px':>11s}{'距离mm':>9s}")
for err, f, tilt, area, dist_ in sorted(rows, reverse=True):
    print(f"{f[-24:]:28s}{err:8.2f}{tilt:7.1f}{area:11.0f}{dist_:9.0f}")

print("\n倾角分布:", " ".join(f"{r[2]:.0f}" for r in sorted(rows, key=lambda x: -x[2])))
max_tilt = max(r[2] for r in rows)
print(f"最大倾角 {max_tilt:.1f}°  (建议至少 2-3 张 >25°)")

# 剔除误差最大的 1/4 再算
n_drop = len(rows) // 4
keep_idx = [i for i, r in enumerate(rows) if i not in {j for j in np.argsort([r[0] for r in rows])[-n_drop:]}]
obj2 = [obj_all[i] for i in keep_idx]; img2 = [img_all[i] for i in keep_idx]
ret2, K2, dist2, _, _ = cv2.calibrateCamera(obj2, img2, shapes[0][::-1], None, None)
print(f"\n剔除误差最大的 {n_drop} 张后（剩 {len(keep_idx)} 张）: RMS {ret2:.4f} px")
print("K2 =\n", K2)
