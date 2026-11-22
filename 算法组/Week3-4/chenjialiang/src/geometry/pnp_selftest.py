# 合成数据自测：验证 PnP 解算与坐标系变换
import cv2

import numpy as np

FIELD_POINTS = np.array([
    [0.0, 0.0, 0.0],
    [4.0, 0.0, 0.0],
    [4.0, 3.0, 0.0],
    [0.0, 3.0, 0.0],
    [1.0, 1.5, 0.0],
    [3.0, 1.5, 0.0],
], dtype=np.float64)

K = np.array([
    [700.0, 0.0, 480.0],
    [0.0, 700.0, 270.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

DIST = np.zeros(5)

CAM_POS = np.array([-3.0, 1.5, 1.5])
CAM_YAW_DEG = 0.0
CAM_PITCH_DEG = 20.0

def build_camera_pose(pos, yaw_deg, pitch_deg):

    yaw = np.deg2rad(yaw_deg)
    pitch = np.deg2rad(pitch_deg)

    forward = np.array([
        np.cos(pitch) * np.cos(yaw),
        np.cos(pitch) * np.sin(yaw),
        -np.sin(pitch),
    ])

    right = np.array([np.sin(yaw), -np.cos(yaw), 0.0])

    down = np.cross(forward, right)

    R = np.array([right, down, forward])

    t = -R @ pos

    rvec, _ = cv2.Rodrigues(R)

    return rvec, t.reshape(3, 1)

def project(points_3d, rvec, tvec):
    pts, _ = cv2.projectPoints(points_3d, rvec, tvec, K, DIST)

    return pts.reshape(-1, 2)

def solve(points_3d, pixels):
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)

    retval, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        points_3d, pixels, K, DIST, flags=cv2.SOLVEPNP_IPPE)

    for i in range(retval):
        R, _ = cv2.Rodrigues(rvecs[i])

        C = -R.T @ tvecs[i]

        if C[2] > 0:
            return rvecs[i], tvecs[i]

    return rvecs[0], tvecs[0]

def camera_center(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)

    return (-R.T @ tvec).flatten()

def main():
    rvec_true, tvec_true = build_camera_pose(CAM_POS, CAM_YAW_DEG, CAM_PITCH_DEG)

    print("=== 设定的真值 ===")
    print("位置:", CAM_POS, " yaw:", CAM_YAW_DEG, "度  pitch:", CAM_PITCH_DEG, "度")

    pixels = project(FIELD_POINTS, rvec_true, tvec_true)

    print("\n=== 投影得到的像素坐标（这是'假装的检测结果'）===")

    for i, (u, v) in enumerate(pixels):
        print(f"  点{i} {FIELD_POINTS[i]} -> 像素 ({u:7.2f}, {v:7.2f})")

    inside = np.all((pixels[:, 0] >= 0) & (pixels[:, 0] <= 960) &
                    (pixels[:, 1] >= 0) & (pixels[:, 1] <= 540))
    print("  全部落在画面内:", inside)

    rng = np.random.default_rng(0)

    pixels_noisy = pixels + rng.normal(0.0, 0.5, pixels.shape)

    rvec_est, tvec_est = solve(FIELD_POINTS, pixels_noisy)
    center_est = camera_center(rvec_est, tvec_est)

    print("\n=== 解出的结果 ===")
    print("位置:", np.round(center_est, 4))

    pos_err = float(np.linalg.norm(center_est - CAM_POS))
    print(f"位置误差: {pos_err:.4f} m")

    R_est, _ = cv2.Rodrigues(rvec_est)

    axis = (R_est.T @ np.array([[0.0], [0.0], [1.0]])).flatten()
    yaw_est = np.degrees(np.arctan2(axis[1], axis[0]))
    print(f"解出 yaw: {yaw_est:.3f} 度")

    yaw_err = abs((yaw_est - CAM_YAW_DEG + 180) % 360 - 180)
    print(f"yaw 误差: {yaw_err:.3f} 度")

    print("\n=== 判定 ===")
    ok_pos = pos_err < 0.01
    ok_yaw = yaw_err < 1.0
    print(f"位置 {'通过' if ok_pos else '不合格'}（阈值 1 cm）")
    print(f"yaw  {'通过' if ok_yaw else '不合格'}（阈值 1 度）")

    if not ok_pos and not ok_yaw:
        print("\n两个都错，先检查 build_camera_pose 里的旋转矩阵和平移向量")

if __name__ == "__main__":
    main()
