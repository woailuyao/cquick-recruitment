import cv2
import numpy as np
import math
from ultralytics import YOLO

# ================= 1. 硬件参数 =================
fx, fy = 1627.48, 1646.78
cx, cy = 677.89, 346.82
CAMERA_HEIGHT = 0.65
PITCH_ANGLE = 15.0

model = YOLO('marker_best.pt')

# ================= 2. 全象限地图库 =================
FIELD_MAP = [
    {"id": "X_1", "type": "XCross", "x": 0.0,  "y": 0.75},
    {"id": "T_1", "type": "TCross", "x": 0.0,  "y": 3.0},
    {"id": "L_1", "type": "LCross", "x": 4.0,  "y": 3.0},
    {"id": "T_2", "type": "TCross", "x": 4.0,  "y": 2.0},
    {"id": "T_3", "type": "TCross", "x": 4.0,  "y": 1.5},
    {"id": "L_2", "type": "LCross", "x": 2.05, "y": 2.0},
    {"id": "L_3", "type": "LCross", "x": 3.05, "y": 1.5},
    {"id": "X_2", "type": "XCross", "x": 0.0,  "y": -0.75},
    {"id": "T_4", "type": "TCross", "x": 0.0,  "y": -3.0},
    {"id": "L_4", "type": "LCross", "x": 4.0,  "y": -3.0},
    {"id": "T_5", "type": "TCross", "x": 4.0,  "y": -2.0},
    {"id": "T_6", "type": "TCross", "x": 4.0,  "y": -1.5},
    {"id": "L_5", "type": "LCross", "x": 2.05, "y": -2.0},
    {"id": "L_6", "type": "LCross", "x": 3.05, "y": -1.5},
    {"id": "L_7", "type": "LCross", "x": -4.0, "y": 3.0},
    {"id": "T_7", "type": "TCross", "x": -4.0, "y": 2.0},
    {"id": "T_8", "type": "TCross", "x": -4.0, "y": 1.5},
    {"id": "L_8", "type": "LCross", "x": -2.05,"y": 2.0},
    {"id": "L_9", "type": "LCross", "x": -3.05,"y": 1.5},
    {"id": "L_10","type": "LCross", "x": -4.0, "y": -3.0},
    {"id": "T_9", "type": "TCross", "x": -4.0, "y": -2.0},
    {"id": "T_10","type": "TCross", "x": -4.0, "y": -1.5},
    {"id": "L_11","type": "LCross", "x": -2.05,"y": -2.0},
    {"id": "L_12","type": "LCross", "x": -3.05,"y": -1.5}
]

# ================= 3. 构建地图画布 =================
MAP_W, MAP_H = 480, 640 
SCALE = 65 
ORIGIN_X = MAP_W // 2
ORIGIN_Y = MAP_H // 2

def map_to_pixel(x, y):
    px = int(ORIGIN_X - y * SCALE)
    py = int(ORIGIN_Y - x * SCALE)
    return px, py

def create_pro_map():
    m = np.zeros((MAP_H, MAP_W, 3), dtype=np.uint8)
    color = (150, 150, 150)
    top_left = map_to_pixel(4, 3)
    bottom_right = map_to_pixel(-4, -3)
    cv2.rectangle(m, top_left, bottom_right, color, 2)
    px_x_axis_left = map_to_pixel(0, 3)
    px_x_axis_right = map_to_pixel(0, -3)
    cv2.line(m, px_x_axis_left, px_x_axis_right, color, 2)
    cv2.circle(m, (ORIGIN_X, ORIGIN_Y), int(0.75 * SCALE), color, 2)
    cv2.rectangle(m, map_to_pixel(4, 2), map_to_pixel(2.05, -2), color, 2)
    cv2.rectangle(m, map_to_pixel(4, 1.5), map_to_pixel(3.05, -1.5), color, 2)
    cv2.rectangle(m, map_to_pixel(-2.05, 2), map_to_pixel(-4, -2), color, 2)
    cv2.rectangle(m, map_to_pixel(-3.05, 1.5), map_to_pixel(-4, -1.5), color, 2)
    for marker in FIELD_MAP:
        px, py = map_to_pixel(marker["x"], marker["y"])
        cv2.circle(m, (px, py), 4, (0, 255, 255), -1)
        cv2.putText(m, marker["type"][0], (px+5, py-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.putText(m, "field.yaml", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(m, "+X (Goal)", (ORIGIN_X + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(m, "+Y", (10, ORIGIN_Y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(m, "(0,0)", (ORIGIN_X + 5, ORIGIN_Y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return m

map_base = create_pro_map()

def pixel_to_ground(u, v):
    theta = math.radians(PITCH_ANGLE)
    alpha = theta + math.atan2(v - cy, fy)
    if alpha <= 0: return None, None 
    distance_forward = CAMERA_HEIGHT / math.tan(alpha)
    distance_left = distance_forward * (u - cx) / fx
    return distance_forward, distance_left

def solve_pose_svd(obs_pts, map_pts):
    obs = np.array(obs_pts)
    map_pts = np.array(map_pts)
    centroid_obs = np.mean(obs, axis=0)
    centroid_map = np.mean(map_pts, axis=0)
    obs_centered = obs - centroid_obs
    map_centered = map_pts - centroid_map
    H = obs_centered.T @ map_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_map - R @ centroid_obs
    yaw = math.atan2(R[1, 0], R[0, 0])
    transformed = (R @ obs_centered.T).T + centroid_map
    residuals = np.linalg.norm(transformed - map_pts, axis=1)
    rmse = np.sqrt(np.mean(residuals**2))
    return t[0], t[1], yaw, rmse

# ================= 5. 主循环 (带图像增强) =================
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened() or not cap.read()[0]:
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    ret, frame = cap.read()
    if not ret: break
    
    # 💡 物理外挂：自适应提亮画面（解决现场光线暗导致的漏检）
    # alpha 控制对比度，beta 控制亮度
    frame = cv2.convertScaleAbs(frame, alpha=1.2, beta=30)

    # 放大左边画面用于显示
    frame_display = cv2.resize(frame, (MAP_W, MAP_H))
    # 推理画面（可以稍微裁剪中心区域，减少干扰）
    frame_infer = cv2.resize(frame, (MAP_W, MAP_H))
    
    # 💡 物理外挂：把 conf 降到极低 0.08，先保证能框出来
    results = model.predict(frame_infer, conf=0.08, verbose=False)
    boxes = results[0].boxes
    
    obs_pts = []
    map_pts = []
    matched_ids = []
    valid = False
    cam_x, cam_y, yaw, residual, marker_count, status = 0.0, 0.0, 0.0, 0.0, 0, "lost"

    if boxes is not None and len(boxes) > 0:
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            u, v = (x1 + x2) / 2, (y1 + y2) / 2
            cls_name = model.names[int(box.cls[0].cpu().numpy())]
            rel_x, rel_y = pixel_to_ground(u, v)
            
            if rel_x is not None:
                # 简单寻找最匹配的点
                for m in FIELD_MAP:
                    if m["type"] == cls_name:
                        obs_pts.append([rel_x, rel_y])
                        map_pts.append([m["x"], m["y"]])
                        matched_ids.append(m["id"])
                        break
                cv2.rectangle(frame_display, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame_display, (int(u), int(v)), 5, (0, 0, 255), -1)

    if len(obs_pts) >= 2:
        marker_count = len(obs_pts)
        cam_x, cam_y, yaw_rad, residual = solve_pose_svd(obs_pts, map_pts)
        yaw = math.degrees(yaw_rad)
        valid = True
        status = "ok"
    elif len(obs_pts) == 1:
        marker_count = 1
        cam_x = map_pts[0][0] - obs_pts[0][0]
        cam_y = map_pts[0][1] - obs_pts[0][1]
        yaw = 0.0
        residual = 0.0
        valid = True
        status = "ok_single_marker"

    # 左上角参数
    cv2.putText(frame_display, f"Intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.putText(frame_display, f"Extrinsics: H={CAMERA_HEIGHT}m, Pitch={PITCH_ANGLE}deg", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    map_display = map_base.copy()
    if valid:
        for mid in matched_ids:
            for m in FIELD_MAP:
                if m["id"] == mid:
                    hx, hy = map_to_pixel(m["x"], m["y"])
                    cv2.circle(map_display, (hx, hy), 8, (0, 255, 0), 2)
        
        px, py = map_to_pixel(cam_x, cam_y)
        end_x = int(px - 30 * math.sin(math.radians(yaw)))
        end_y = int(py - 30 * math.cos(math.radians(yaw)))
        cv2.circle(map_display, (px, py), 6, (0, 0, 255), -1)
        cv2.arrowedLine(map_display, (px, py), (end_x, end_y), (0, 0, 255), 2)
        
        cv2.putText(map_display, f"x_camera_map: {cam_x:.2f} m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(map_display, f"y_camera_map: {cam_y:.2f} m", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(map_display, f"yaw_camera_map: {yaw:.1f} deg", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(map_display, f"marker_count: {marker_count}", (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(map_display, f"residual: {residual:.2f}", (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(map_display, f"valid: True, status: {status}", (10, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(map_display, f"Matched ID: {', '.join(matched_ids)}", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    else:
        cv2.putText(map_display, "x_camera_map: NaN", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(map_display, "y_camera_map: NaN", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(map_display, "yaw_camera_map: NaN", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(map_display, "marker_count: 0", (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(map_display, "residual: 0.00", (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(map_display, "valid: False, status: LOST", (10, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    combined = np.hstack((frame_display, map_display))
    cv2.imshow("Task 2 Full Compliance", combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()