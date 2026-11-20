import cv2
import numpy as np
from ultralytics import YOLO
import math
import time

# ================= 1. 加载模型与相机内参 =================

ball_model = YOLO("ball_best.pt")
marker_model = YOLO("marker_best.pt")
calib_data = np.load("camera_calibration.npz")
mtx = calib_data['mtx']
dist = calib_data['dist']

# ================= 2. 物理外参 (现场实测) =================
CAMERA_HEIGHT = 0.665  # 相机高度 0.665米
CAMERA_PITCH_DEG = 29.0 # 相机俯仰角 29度 (通过水平仪实测校准)
pitch_rad = np.radians(CAMERA_PITCH_DEG)
# 绕X轴的旋转矩阵 (将相机坐标系转为载体坐标系)
R = np.array([
    [1, 0, 0], 
    [0, np.cos(pitch_rad), -np.sin(pitch_rad)], 
    [0, np.sin(pitch_rad), np.cos(pitch_rad)]
])

# ================= 3. 场地地图 (现场实测校准版, 单位: 米) =================
FIELD_MAP = {
    "XCross": [
        [0.000, 0.745], 
        [0.000, -0.745]
    ],
    "TCross": [
        [0.000, 3.000], [0.000, -3.000],
        [3.900, 1.980], [3.900, -1.980],
        [3.900, 1.460], [3.900, -1.460]
    ],
    "LCross": [
        [3.900, 3.000], [3.900, -3.000],
        [2.000, 1.980], [2.000, -1.980],
        [2.960, 1.460], [2.960, -1.460],
        [-3.900, 3.000], [-3.900, -3.000]
    ]
}

# ================= 4. 打开摄像头 =================
# 在Windows下，外接USB摄像头通常为1 (如果报错可尝试改为0或2)
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret: break
    
    # 去畸变
    frame_undistorted = cv2.undistort(frame, mtx, dist)
    
    # --- 任务1：识别足球 ---
    ball_results = ball_model(frame_undistorted, verbose=False, conf=0.25)
    frame_display = ball_results[0].plot() # 在画面上画出足球框
    
    # --- 任务2：识别场地标志 ---
    marker_results = marker_model(frame_undistorted, verbose=False, conf=0.25)
    obs_markers = []
    for box in marker_results[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        u, v = (x1 + x2) / 2, (y1 + y2) / 2
        cls_name = marker_model.names[int(box.cls[0])]
        
        # 画标志检测框
        cv2.rectangle(frame_display, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
        cv2.putText(frame_display, cls_name, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # 像素点 -> 地面坐标
        ray_cam = np.linalg.inv(mtx) @ np.array([u, v, 1.0])
        ray_body = R @ ray_cam
        if ray_body[2] > 0:
            lam = CAMERA_HEIGHT / ray_body[2]
            # 核心修改：Y轴镜像翻转 (-ray_body[1])
            obs_markers.append({"type": cls_name, "x": ray_body[0]*lam, "y": -ray_body[1]*lam})

    # --- 计算实时 FPS ---
    curr_time = time.time()
    fps = 1.0 / (curr_time - prev_time)
    prev_time = curr_time
    cv2.putText(frame_display, f"FPS: {fps:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    # ================= 5. 核心匹配算法 =================
    valid = False
    best_pose = None
    best_yaw = 0.0
    min_error = 999.0
    status_text = "LOST"

    # 情况一：>= 2 个点，解算 Yaw + X/Y (多点消除多解性)
    if len(obs_markers) >= 2:
        for yaw_deg in range(0, 360, 5):
            yaw_rad = math.radians(yaw_deg)
            cos_y = math.cos(yaw_rad)
            sin_y = math.sin(yaw_rad)
            
            rotated_obs = []
            for obs in obs_markers:
                rx = obs["x"] * cos_y - obs["y"] * sin_y
                ry = obs["x"] * sin_y + obs["y"] * cos_y
                rotated_obs.append({"type": obs["type"], "x": rx, "y": ry})
            
            first_obs = rotated_obs[0]
            for map_pt in FIELD_MAP.get(first_obs["type"], []):
                tx = map_pt[0] - first_obs["x"]
                ty = map_pt[1] - first_obs["y"]
                
                current_error = 0
                match_count = 0
                for r_obs in rotated_obs:
                    test_map_pts = FIELD_MAP.get(r_obs["type"], [])
                    min_dist_to_map = 999.0
                    for test_pt in test_map_pts:
                        dist_to_map = math.hypot((r_obs["x"] + tx) - test_pt[0], (r_obs["y"] + ty) - test_pt[1])
                        if dist_to_map < min_dist_to_map:
                            min_dist_to_map = dist_to_map
                    if min_dist_to_map < 2.0:
                        current_error += min_dist_to_map
                        match_count += 1
                
                if match_count == len(rotated_obs) and (current_error / match_count) < min_error:
                    min_error = current_error / match_count
