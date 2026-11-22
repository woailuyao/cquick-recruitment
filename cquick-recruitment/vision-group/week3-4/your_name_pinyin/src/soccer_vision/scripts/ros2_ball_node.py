#!/usr/bin/env python3
"""ROS2 节点：足球检测，发布 /soccer/ball。

用法：
    ros2 run soccer_vision ros2_ball_node --ros-args -p weights:=best.pt
"""

import sys
from pathlib import Path

# 确保能 import 到同包的 Python 模块（soccer_vision 库 + 生成的消息）
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from soccer_vision.msg import BallDetection

import cv2
from ultralytics import YOLO


class BallNode(Node):
    def __init__(self):
        super().__init__("ball_detection_node")
        self.declare_parameter("weights", "best.pt")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("conf", 0.5)

        weights = self.get_parameter("weights").value
        device = self.get_parameter("device").value
        conf = self.get_parameter("conf").value

        self.model = YOLO(weights)
        self.pub = self.create_publisher(BallDetection, "/soccer/ball", 10)

        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开相机")
            raise RuntimeError("camera open failed")

        self.conf = conf
        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info("ball_detection_node 启动")

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        results = self.model(frame, conf=self.conf, verbose=False)
        boxes = results[0].boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            name = results[0].names[cls]
            if name != "ball":
                continue
            msg = BallDetection()
            msg.header = Header(stamp=self.get_clock().now().to_msg())
            msg.confidence = conf
            msg.x1, msg.y1, msg.x2, msg.y2 = x1, y1, x2, y2
            msg.center_u = (x1 + x2) / 2.0
            msg.center_v = (y1 + y2) / 2.0
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = BallNode()
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
