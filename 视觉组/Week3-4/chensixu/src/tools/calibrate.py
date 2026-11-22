import cv2
import numpy as np
import glob
import os

print("开始寻找图片...")

# 棋盘格内部角点数量 (横向5个，纵向7个)
chessboard_size = (5, 7) 

# 读取 calib_imgs 文件夹下的所有 jpg 图片
images = glob.glob('calib_imgs/*.jpg') 
if len(images) == 0:
    print("❌ 错误：在 data 文件夹里没找到图片！请检查路径。")
    exit()

# 找角点的参数
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

objpoints = [] # 3d点
imgpoints = [] # 2d点
valid_images = 0

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 找棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    if ret == True:
        objpoints.append(objp)
        # 提高精度
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        valid_images += 1
        print(f"✅ 成功找到角点: {fname}")
    else:
        print(f"❌ 没找到角点 (可能是反光、没拍全或糊了): {fname}")

# 开始标定
if len(objpoints) > 0:
    print(f"\n正在用 {valid_images} 张有效图片进行计算...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    print("\n================ 标定结果 ================")
    print(f"重投影误差 (越低越好，小于 0.5 算优秀): {ret:.4f}")
    
    print(f"\n相机内参矩阵 K:\n{mtx}")
    
    print(f"\n畸变系数 dist:\n{dist}")
    
    print("\n👉 请把上面的矩阵和系数抄下来，或者截图保存！")
else:
    print("\n❌ 没有足够的有效图片用来标定！请重新拍！")