import cv2
import numpy as np
import math
import yaml
import time
from ultralytics import YOLO

# ================= 🚨 明天实地录制前，确认参数 =================
MODEL_PATH = r"D:\Marker_Dataset\runs\train_marker-2\weights\best.pt"
INTRINSICS_YAML = r"D:\Marker_Dataset\camera_intrinsics.yaml"
CAMERA_HEIGHT = 0.5  # 相机高度
CAMERA_PITCH_DEG = 15.0  # 相机俯角
CAMERA_ID = 0  # 摄像头ID
# ============================================================

# 1. 加载地图数据 (明天记得替换成组长发的真实 yaml 里的数据)
field_map_data = {
    'TCross1': [5.955, 2.930], 'TCross2': [4.945, 2.930], 'TCross3': [1.000, 2.930], 'TCross4': [0.000, 2.930],
    'LCross1': [7.465, 2.930], 'LCross2': [-1.485, 2.930], 'LCross3': [4.945, 2.020], 'LCross4': [1.000, 2.020],
    'LCross5': [5.955, 0.000],
    'XCross1': [4.505, -4.010], 'XCross2': [1.565, -4.010], 'PenaltyPoint': [2.965, 0.885]
}

FX, FY, CX, CY = 800.0, 800.0, 640.0, 360.0


def load_intrinsics():
    global FX, FY, CX, CY
    try:
        with open(INTRINSICS_YAML, 'r') as f:
            mtx = yaml.safe_load(f)['camera_matrix']
            FX, FY, CX, CY = mtx[0][0], mtx[1][1], mtx[0][2], mtx[1][2]
    except:
        pass


def pixel_to_ground(u, v):
    x_c, y_c = (u - CX) / FX, (v - CY) / FY
    ray_camera = np.array([x_c, y_c, 1.0])
    pitch_rad = math.radians(CAMERA_PITCH_DEG)
    cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)
    R_x = np.array([[1, 0, 0], [0, cos_p, sin_p], [0, -sin_p, cos_p]])
    ray_ground = np.dot(R_x, ray_camera)
    dy = ray_ground[1]
    if dy <= 0.001: return None, None
    lam = CAMERA_HEIGHT / dy
    return lam * ray_ground[2], lam * ray_ground[0]


def solve_pose_from_two_points(p1_obs, p2_obs, p1_map, p2_map):
    vec_obs = np.array([p2_obs[0] - p1_obs[0], p2_obs[1] - p1_obs[1]])
    vec_map = np.array([p2_map[0] - p1_map[0], p2_map[1] - p1_map[1]])
    angle_obs, angle_map = math.atan2(vec_obs[1], vec_obs[0]), math.atan2(vec_map[1], vec_map[0])
    yaw_rad = angle_map - angle_obs
    cos_a, sin_a = math.cos(yaw_rad), math.sin(yaw_rad)
    R = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
    t = np.array(p1_map) - np.dot(R, np.array(p1_obs))
    return t[0], t[1], math.degrees(yaw_rad)


def match_and_localize(observed_markers):
    best_pose, min_error = None, float('inf')
    if len(observed_markers) < 2: return None, None
    for i in range(len(observed_markers)):
        for j in range(i + 1, len(observed_markers)):
            m1_type, m1_x, m1_y = observed_markers[i]
            m2_type, m2_x, m2_y = observed_markers[j]
            map_pts1 = {k: v for k, v in field_map_data.items() if k.startswith(m1_type)}
            map_pts2 = {k: v for k, v in field_map_data.items() if k.startswith(m2_type)}
            for name1, coord1 in map_pts1.items():
                for name2, coord2 in map_pts2.items():
                    if name1 == name2: continue
                    cam_x, cam_y, cam_yaw = solve_pose_from_two_points((m1_x, m1_y), (m2_x, m2_y), coord1, coord2)
                    total_error = 0
                    for m_type, mx, my in observed_markers:
                        cos_a, sin_a = math.cos(math.radians(cam_yaw)), math.sin(math.radians(cam_yaw))
                        R = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
                        p_pred = np.dot(R, np.array([mx, my])) + np.array([cam_x, cam_y])
                        min_dist = min([np.linalg.norm(p_pred - np.array(v)) for k, v in field_map_data.items() if
                                        k.startswith(m_type)])
                        total_error += min_dist
                    if total_error < min_error:
                        min_error, best_pose = total_error, (cam_x, cam_y, cam_yaw)
    return best_pose, min_error


def main():
    load_intrinsics()
    model = YOLO(MODEL_PATH)
    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    prev_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret: break
        fps = 1 / (time.time() - prev_time)
        prev_time = time.time()

        results = model(frame, conf=0.5, verbose=False)
        boxes = results[0].boxes
        annotated_frame = results[0].plot()

        # 收集当前帧所有有效标志的距离
        observed_markers = []
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
            cls_name = model.names[int(box.cls[0])]
            X, Y = pixel_to_ground(u, v)

            if X is not None and X > 0:
                # 这里假设我们观测到标志，记录下来给阶段三用
                # 注意：为了符合标准数学系，我们将向下的 Y 取反（变成向上为正）
                observed_markers.append((cls_name, X, -Y))
                cv2.putText(annotated_frame, f"{cls_name}: Fwd {X:.1f}m", (u - 50, int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # ================= 调用地图匹配 =================
        pose, err = match_and_localize(observed_markers)

        # 界面左上角画信息面板
        overlay = annot