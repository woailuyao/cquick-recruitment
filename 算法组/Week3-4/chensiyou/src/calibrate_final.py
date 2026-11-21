import cv2
import numpy as np
import glob
import os
import yaml

CHESSBOARD_SIZE = (9, 6)

SQUARE_SIZE = 0.029

IMAGES_DIR = r"D:\Marker_Dataset\calibration_image"


images = glob.glob(os.path.join(IMAGES_DIR, '*.jpg')) + glob.glob(os.path.join(IMAGES_DIR, '*.png'))
print(f"共找到 {len(images)} 张图片，开始处理...")


objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

found_count = 0
for fname in images:
    img = cv2.imread(fname)
    if img is None: continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)
        found_count += 1
        print(f"成功提取角点: {os.path.basename(fname)}")
    else:
        print(f" 失败(没找到角点): {os.path.basename(fname)}")

if found_count >= 10:
    print("\n正在计算内参...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("\n标定成功！结果如下 ")
    print(f"重投影误差: {ret:.4f} (要求小于0.5)")
    print("\n内参矩阵 K:")
    print(mtx)
    print("\n畸变系数 dist:")
    print(dist)
    

    # 自动保存为 YAML 文件
    yaml_path = r"D:\Marker_Dataset\camera_intrinsics.yaml"
    camera_data = {
        'camera_matrix': mtx.tolist(),
        'dist_coeff': dist.tolist(),
        'image_width': gray.shape[1],
        'image_height': gray.shape[0],
        'reprojection_error': float(ret)
    }
    with open(yaml_path, 'w') as f:
        yaml.dump(camera_data, f, default_flow_style=False)

    print(f"\n 参数已自动保存至: {yaml_path}")
else:
    print(f"\n 只找到了 {found_count} 张图的有效角点，数量太少，请重新拍照！")