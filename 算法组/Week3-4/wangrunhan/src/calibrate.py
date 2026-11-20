import cv2
import numpy as np
import glob
import os

# 1. 设置棋盘格参数 (8x5 内角点)
pattern_size = (8, 5) 
# 格子实际边长（米），按你当时量的填
square_size = 0.032 

objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp = objp * square_size

objpoints = [] 
imgpoints = [] 

# 2. 读取图片
images = glob.glob('calib_images/*.jpg') + glob.glob('calib_images/*.png')
print(f"找到 {len(images)} 张标定图片，开始处理...")

found_count = 0
for fname in images:
    img = cv2.imread(fname)
    if img is None: continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)
        found_count += 1

# 3. 计算并保存
if found_count > 0:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("\n✅ 标定完成！")
    print("内参矩阵 (mtx):\n", mtx)
    print("畸变系数 (dist):\n", dist)
    np.savez("camera_calibration.npz", mtx=mtx, dist=dist)
    print("参数已保存到 camera_calibration.npz")
else:
    print("❌ 标定失败，未找到棋盘格角点。")