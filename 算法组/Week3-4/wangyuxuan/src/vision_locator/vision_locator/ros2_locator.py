#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import FieldPose 

import cv2
import numpy as np
import math
import yaml
import time 
from ultralytics import YOLO

# ================= 0. 场地真实坐标数据 =================
FIELD_MAP = {
    "TCross": [[0,3],[0,-3],[3.9,2],[3.9,1.48],[3.9,-2],[3.9,-1.48],[-3.9,2],[-3.9,1.48],[-3.9,-2],[-3.9,-1.48]],
    "LCross": [[2,2],[2.96,1.48],[3.9,3],[2,-2],[2.96,-1.48],[3.9,-3],[-2,2],[-2.96,1.48],[-3.9,3],[-2,-2],[-2.96,-1.48],[-3.9,-3]],
    "XCross": [[0,-0.73],[0,0.73]]
}

MODEL_PATH = "/home/capybarawyx/vision_ws/k1_soccer_vision/runs/detect/train-marker/weights/best.pt"
YAML_PATH = "/home/capybarawyx/vision_ws/k1_soccer_vision/answer_commit/calibration/camera_extrinsics.yaml"
K = np.array([[1612.39339, 0.0, 655.892693], [0.0, 1604.86671, 370.489463], [0.0, 0.0, 1.0]])

# ================= 1. 数学与匹配函数 =================
def pixel_to_ground(u, v, K, height, pitch_deg):
    uv_1 = np.array([[u], [v], [1.0]])
    K_inv = np.linalg.inv(K)
    ray_cam = K_inv.dot(uv_1)
    pitch_rad = np.radians(pitch_deg)
    xc, yc, zc = ray_cam[0, 0], ray_cam[1, 0], ray_cam[2, 0]
    forward = zc * np.cos(pitch_rad) - yc * np.sin(pitch_rad)
    downward = zc * np.sin(pitch_rad) + yc * np.cos(pitch_rad)
    if downward <= 0.001: return None
    scale = height / downward
    return scale * forward, scale * xc

def estimate_pose(observed_markers):
    if len(observed_markers) < 2: 
        return None, "insufficient_markers", 999.0
    m1, m2 = observed_markers[0], observed_markers[1]
    if m1[0] == m2[0]: return None, "ambiguous", 999.0
    
    best_error = float('inf')
    best_pose = None
    for wx1, wy1 in FIELD_MAP.get(m1[0], []):
        for wx2, wy2 in FIELD_MAP.get(m2[0], []):
            map_dist = math.hypot(wx2 - wx1, wy2 - wy1)
            map_angle = math.atan2(wy2 - wy1, wx2 - wx1)
            obs_dist = math.hypot(m2[1] - m1[1], m2[2] - m1[2])
            obs_angle = math.atan2(m2[2] - m1[2], m2[1] - m1[1])
            if abs(map_dist - obs_dist) > 1.5: continue
                
            map_yaw = map_angle - obs_angle
            map_x = wx1 - (m1[1] * math.cos(map_yaw) - m1[2] * math.sin(map_yaw))
            map_y = wy1 - (m1[1] * math.sin(map_yaw) + m1[2] * math.cos(map_yaw))
            
            error, valid_markers = 0, 0
            for name, ox, oy in observed_markers:
                pred_x = map_x + ox * math.cos(map_yaw) - oy * math.sin(map_yaw)
                pred_y = map_y + ox * math.sin(map_yaw) + oy * math.cos(map_yaw)
                min_dist = float('inf')
                for rx, ry in FIELD_MAP.get(name, []):
                    d = math.hypot(pred_x - rx, pred_y - ry)
                    if d < min_dist: min_dist = d
                if min_dist < 1.5:
                    error += min_dist
                    valid_markers += 1
            if valid_markers > 0:
                error /= valid_markers
                if error < best_error:
                    best_error = error
                    best_pose = (map_x, map_y, math.degrees(map_yaw) % 360)
    
    if best_pose and best_error > 1.0: return best_pose, "high_residual", best_error
    elif best_pose: return best_pose, "ok", best_error
    return None, "lost", 999.0

# ================= 2. ROS 2 节点类 =================
class LocatorNode(Node):
    def __init__(self):
        super().__init__('vision_locator_node')
        self.pub = self.create_publisher(FieldPose, '/localization/field_pose', 10)
        
        self.model = YOLO(MODEL_PATH)
        with open(YAML_PATH, 'r') as f:
            ext = yaml.safe_load(f)
            self.h = ext['camera_height_m']
            self.p = ext['pitch_deg']
            
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('✅ 节点已启动！按 Ctrl+C 退出。')

    def timer_callback(self):
        start_time = time.time() # 记录时间算 FPS
        
        ret, frame = self.cap.read()
        if not ret: return

        results = self.model(frame, verbose=False)
        annotated_frame = frame.copy()
        observed = []
        #置信度
        for box in results[0].boxes:
            if box.conf[0] > 0.5:
                cls_id = int(box.cls[0].item())
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                
                if cls_id == 0: name = "TCross"
                elif cls_id == 1: name = "LCross"
                elif cls_id == 2: name = "XCross"
                else: continue
                
                u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
                res = pixel_to_ground(u, v, K, self.h, self.p)
                
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(annotated_frame, (u, v), 5, (0, 0, 255), -1)
                
                if res is not None:
                    observed.append((name, res[0], res[1]))
                    cv2.putText(annotated_frame, f"{name} X={res[0]:.1f}", (u+10, v-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # ================= 3. 构造消息并发布 =================
        msg = FieldPose()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.marker_count = len(observed)
        
        if len(observed) >= 2:
            pose, status, residual = estimate_pose(observed)
            msg.status = status
            msg.residual = float(residual)
            
            if pose is not None:
                msg.valid = True
                msg.x_camera_map = float(pose[0])
                msg.y_camera_map = float(pose[1])
                msg.yaw_camera_map = float(pose[2])
                
                cv2.putText(annotated_frame, f"Map: X={msg.x_camera_map:.2f}, Y={msg.y_camera_map:.2f}", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                self.get_logger().info(f'定位成功: X={msg.x_camera_map:.2f}, Y={msg.y_camera_map:.2f}')
            else:
                msg.valid = False
                cv2.putText(annotated_frame, f"Status: {status}", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                self.get_logger().warn(f'定位失败: {status}')
        else:
            msg.valid = False
            msg.status = "insufficient_markers"
            msg.residual = 999.0
            cv2.putText(annotated_frame, "SEARCHING...", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.get_logger().warn('标志不足 (insufficient_markers)')

        self.pub.publish(msg)

        # ================= 4. 算 FPS 并显示在画面上 =================
        fps = 1.0 / (time.time() - start_time + 1e-5)
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (20, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        cv2.imshow("Locator Camera View", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()