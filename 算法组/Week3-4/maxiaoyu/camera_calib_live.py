import cv2
import numpy as np
import yaml

# 已根据你的棋盘修改：7列×5行内角点，格子34mm
PATTERN_SIZE = (7, 5)
SQUARE_SIZE = 0.034
OUTPUT_YAML = "./config/camera_intrinsics.yaml"

objp = np.zeros((PATTERN_SIZE[0]*PATTERN_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1,2) * SQUARE_SIZE

obj_points = []
img_points = []

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("==== 实时标定 ====")
print("操作：")
print("  s ：保存当前识别成功的棋盘格画面")
print("  q ：退出并开始标定计算")
print("  ESC：直接退出，不保存")

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
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_sub = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            obj_points.append(objp)
            img_points.append(corners_sub)
            print(f"✅ 已保存第 {len(obj_points)} 组角点")
        else:
            print("❌ 当前画面没有识别到棋盘格，无法保存")
    elif key == ord('q'):
        break
    elif key == 27:
        cap.release()
        cv2.destroyAllWindows()
        exit()

cap.release()
cv2.destroyAllWindows()

if len(obj_points) < 10:
    print(f"采集有效图片只有 {len(obj_points)} 组，太少了，至少10组！")
    exit()

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray.shape[::-1], None, None
)

print("\n==== 标定结果 ====")
print("K 内参矩阵：")
print(K)
print("畸变系数：")
print(dist)
print(f"重投影误差：{ret:.4f}")

save_data = {
    "image_width": 1280,
    "image_height": 720,
    "camera_matrix": {
        "fx": float(K[0,0]),
        "fy": float(K[1,1]),
        "cx": float(K[0,2]),
        "cy": float(K[1,2]),
    },
    "dist_coeffs": dist.flatten().tolist(),
    "reprojection_error": float(ret)
}
with open(OUTPUT_YAML, "w", encoding="utf-8") as f:
    yaml.dump(save_data, f, sort_keys=False)
print(f"\n✅ 参数保存到 {OUTPUT_YAML}")


