#!/usr/bin/env python3

import os

import cv2
import rclpy
from rclpy.node import Node
from soccer_interfaces.msg import BallDetection

class BallDetector(Node):
    def __init__(self):
        super().__init__("ball_detector")
        self.declare_parameter("source", "http://192.168.56.1:8080/video")
        self.declare_parameter("model", os.path.expanduser("~/ros2_ws/week3_4/models/ball_best.pt"))
        self.declare_parameter("conf", 0.4)

        self.source = self.get_parameter("source").value
        self.model_path = self.get_parameter("model").value
        self.conf = float(self.get_parameter("conf").value)

        self.pub = self.create_publisher(BallDetection, "/soccer/ball", 10)
        self.model = None
        self._warned = False

        if str(self.source).startswith("http"):
            self.cap = cv2.VideoCapture(str(self.source), cv2.CAP_FFMPEG)
        else:
            self.cap = cv2.VideoCapture(int(self.source), cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.timer = self.create_timer(1.0 / 20.0, self.on_timer)
        self.get_logger().info(f"source={self.source}, model={self.model_path}")

    def _load_model(self):
        if self.model is not None:
            return True
        if not os.path.exists(self.model_path):
            if not self._warned:
                self.get_logger().warn(f"模型不存在：{self.model_path}，先发布 valid=False")
                self._warned = True
            return False
        try:
            from ultralytics import YOLO
        except ImportError:
            if not self._warned:
                self.get_logger().warn("未安装 ultralytics，先发布 valid=False")
                self._warned = True
            return False
        self.model = YOLO(self.model_path)
        return True

    def on_timer(self):
        ok, frame = self.cap.read()
        msg = BallDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        if not ok or frame is None:
            msg.valid = False
            self.pub.publish(msg)
            return

        if not self._load_model():
            msg.valid = False
            self.pub.publish(msg)
            return

        results = self.model.predict(frame, conf=self.conf, verbose=False)
        best = None
        for box in results[0].boxes:
            name = results[0].names.get(int(box.cls[0]), "")
            if name != "ball":
                continue
            conf = float(box.conf[0])
            if best is None or conf > best[0]:
                best = (conf, box.xyxy[0].tolist())

        if best is None:
            msg.valid = False
            self.pub.publish(msg)
            return

        x1, y1, x2, y2 = best[1]
        msg.valid = True
        msg.confidence = best[0]
        msg.x = float(x1)
        msg.y = float(y1)
        msg.width = float(x2 - x1)
        msg.height = float(y2 - y1)
        msg.center_u = float((x1 + x2) / 2.0)
        msg.center_v = float((y1 + y2) / 2.0)
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = BallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
