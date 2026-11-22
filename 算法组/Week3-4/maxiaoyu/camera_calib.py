import cv2
import numpy as np
import yaml

# 棋盘内角点规格
PATTERN_SIZE = (9, 6)
# 棋盘方格边长 34mm = 0.034 米
SQUARE_SIZE = 0.034
OUTPUT_YAML = "./config/camera_intrinsics.yaml"

objp = np.zeros((PATTERN_SIZE[0]*PATTERN_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE

obj_points = []
img_points = []

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("==== 实时标定 ====")
print("操作：")
print("  s : 保存当前识别成功的棋盘格画面")
print("  q : 退出并开始标定计算")
print("ESC : 直接退出，不保存")

while True:
    ret, frame = cap.read()
    if not ret:
        print("读取相机失败！")
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE)

    disp = frame.copy()
    if found:
        cv2.drawChessboardCorners(disp, PATTERN_SIZE, corners, found)

    cv2.imshow("calib", disp)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        # 保存角点
        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER)
            obj_points.append(objp)
            img_points.append(corners2)
            print(f"✅已保存当前棋盘，累计图片数量：{len(obj_points)}")
        else:
            print("❌当前画面识别不到棋盘角点，无法保存！")
    elif key == ord('q'):
        break
    elif key == 27: # ESC
        cap.release()
        cv2.destroyAllWindows()
        exit()

cap.release()
cv2.destroyAllWindows()

if len(obj_points) < 8:
    print(f"⚠️图片数量太少({len(obj_points)}张)，至少8张才能标定！")
else:
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    print("\n========标定结果========")
    print("相机内参矩阵 K：")
    print(K)
    print("\n畸变系数 dist：")
    print(dist)

    # 保存yaml，先创建config文件夹
    import os
    os.makedirs("./config", exist_ok=True)
    intrinsic_data = {
        "K": K.tolist(),
        "distortion": dist.tolist()
    }
    with open(OUTPUT_YAML, "w") as f:
        yaml.dump(intrinsic_data, f)
    print(f"\n✅内参文件保存成功：{OUTPUT_YAML}")
