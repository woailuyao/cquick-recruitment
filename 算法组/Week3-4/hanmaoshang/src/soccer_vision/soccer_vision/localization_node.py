#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from soccer_interfaces.msg import FieldPose
import cv2
import math
import numpy as np
from ultralytics import YOLO

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.publisher_ = self.create_publisher(FieldPose, '/localization/field_pose', 10)
        self.model = YOLO('marker_best.pt')
        self.cap = cv2.VideoCapture(0) # Windows下默认0
        self.fx, self.fy, self.cx, self.cy = 1627.48, 1646.78, 677.89, 346.82
        self.h, self.pitch = 0.65, 15.0
        self.timer = self.create_timer(0.1, self.timer_callback)

    def pixel_to_ground(self, u, v):
        alpha = math.radians(self.pitch) + math.atan2(v - self.cy, self.fy)
        if alpha <= 0: return None, None 
        return self.h / math.tan(alpha), (self.h / math.tan(alpha)) * (u - self.cx) / self.fx

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return
        results = self.model.predict(frame, conf=0.15, verbose=False)
        msg = FieldPose()
        valid = False
        if results[0].boxes is not None and len(results[0].boxes) > 0:
            box = results[0].boxes[0].xyxy[0].cpu().numpy()
            u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            rel_x, rel_y = self.pixel_to_ground(u, v)
            if rel_x is not None:
                msg.x_camera_map = float(0.0 - rel_x)
                msg.y_camera_map = float(0.75 - rel_y)
                msg.yaw_camera_map = 0.0
                msg.marker_count = 1
                msg.residual = 0.0
                msg.valid = True
                valid = True
        if not valid:
            msg.valid = False
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()