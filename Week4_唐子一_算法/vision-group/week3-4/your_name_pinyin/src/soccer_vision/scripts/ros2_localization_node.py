#!/usr/bin/env python3
"""ROS2 节点：场地自定位，发布 /localization/field_pose。

用法：
    ros2 run soccer_vision ros2_localization_node --ros-args \
        -p field:=config/field.yaml \
        -p geometry:=config/field_geometry.yaml \
        -p intrinsics:=calibration/camera_intrinsics.yaml \
        -p extrinsics:=calibration/camera_extrinsics.yaml \
        -p marker_model:=best_marker.pt
"""

import sys
from pathlib import Path

# 确保能 import 到同包的 Python 模块（soccer_vision 库 + 生成的消息）
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from soccer_vision.msg import FieldPose
from soccer_vision.camera_model import CameraGeometry, Extrinsics, Intrinsics
from soccer_vision.field_map import FieldMap
from soccer_vision.landmark_refine import refine
from soccer_vision.localizer import Localizer, Observation

import cv2
from ultralytics import YOLO


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("field_localization_node")
        self.declare_parameter("field", "config/field.yaml")
        self.declare_parameter("geometry", "config/field_geometry.yaml")
        self.declare_parameter("intrinsics", "calibration/camera_intrinsics.yaml")
        self.declare_parameter("extrinsics", "calibration/camera_extrinsics.yaml")
        self.declare_parameter("marker_model", "best_marker.pt")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("conf", 0.5)

        field_p = self.get_parameter("field").value
        geom_p = self.get_parameter("geometry").value
        intr_p = self.get_parameter("intrinsics").value
        extr_p = self.get_parameter("extrinsics").value
        marker_model = self.get_parameter("marker_model").value
        device = self.get_parameter("device").value
        self.conf = self.get_parameter("conf").value

        self.field = FieldMap(field_p, geom_p)
        self.intr = Intrinsics.from_yaml(intr_p)
        self.extr = Extrinsics.from_yaml(extr_p)
        self.cam = CameraGeometry(self.intr, self.extr)
        self.localizer = Localizer(self.field)
        self.model = YOLO(marker_model)

        self.pub = self.create_publisher(FieldPose, "/localization/field_pose", 10)

        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.intr.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.intr.height)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开相机")
            raise RuntimeError("camera open failed")

        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info(
            f"field_localization_node 启动，field v{self.field.version}"
        )

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.model(frame, conf=self.conf, verbose=False)
        boxes = results[0].boxes

        observations = []
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            name = results[0].names[cls]
            lm = refine(gray, name, (x1, y1, x2, y2), conf)
            if lm is None:
                continue
            P = self.cam.project_to_ground(lm.u, lm.v)
            if P is None:
                continue
            observations.append(
                Observation(type=lm.type, x=float(P[0]), y=float(P[1]), confidence=conf)
            )

        pose = self.localizer.localize(observations)

        msg = FieldPose()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.x_camera_map = pose.x
        msg.y_camera_map = pose.y
        msg.yaw_camera_map = pose.yaw_deg
        # 本实现直接把相机光心定义为载体参考原点，两组位姿一致
        msg.x_carrier_map = pose.x
        msg.y_carrier_map = pose.y
        msg.yaw_carrier_map = pose.yaw_deg
        msg.valid = pose.valid
        msg.marker_count = pose.marker_count
        msg.residual = pose.residual
        msg.status = pose.status
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
