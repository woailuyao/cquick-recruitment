#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CQuick 任务一 · 足球检测 ROS2 节点

发布：
    /ball/detections   std_msgs/String   JSON: {"stamp":..., "fps":..., "balls":[{"u":..,"v":..,"conf":..,"w":..,"h":..}]}
    /ball/image        sensor_msgs/Image 可选（publish_image:=true，带宽较大）

参数：
    model_path       best.pt 路径
    source           0 = 相机；或视频/图片路径
    conf             置信度阈值（默认 0.25）
    publish_image    是否发布图像（默认 false）
"""

import json
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BallDetectNode(Node):
    def __init__(self):
        super().__init__("task1_ball_detect")
        self.declare_parameter("model_path", "weights/best.pt")
        self.declare_parameter("source", 0)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("publish_image", False)

        model_path = str(self.get_parameter("model_path").value)
        self.conf = float(self.get_parameter("conf").value)
        self.publish_image = bool(self.get_parameter("publish_image").value)

        try:
            from ultralytics import YOLO
            if not os.path.exists(model_path):
                raise FileNotFoundError(model_path)
            self.model = YOLO(model_path)
            self.get_logger().info(f"加载模型 {model_path}")
        except Exception as e:
            self.get_logger().error(f"模型加载失败（{e}），节点将只发布空结果")
            self.model = None

        src = self.get_parameter("source").value
        src = int(src) if isinstance(src, int) or str(src).isdigit() else str(src)
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            self.get_logger().error(f"打不开视频源 {src}")
        else:
            self.get_logger().info(f"视频源已打开：{src}")

        self.pub_det = self.create_publisher(String, "/ball/detections", 10)
        self.pub_img = None
        if self.publish_image:
            from sensor_msgs.msg import Image
            self.pub_img = self.create_publisher(Image, "/ball/image", 10)

        self.last_t = time.time()
        self.timer = self.create_timer(0.03, self.on_timer)   # ~30 Hz 取帧

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        h, w = frame.shape[:2]
        balls = []
        if self.model is not None:
            res = self.model.predict(frame, conf=self.conf, verbose=False, device="cpu")[0]
            for box in res.boxes:
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                balls.append({
                    "u": round((x1 + x2) / 2.0, 1),
                    "v": round((y1 + y2) / 2.0, 1),
                    "conf": round(float(box.conf[0]), 3),
                    "w": round(x2 - x1, 1),
                    "h": round(y2 - y1, 1),
                })

        now = time.time()
        fps = 1.0 / max(now - self.last_t, 1e-6)
        self.last_t = now

        msg = String()
        msg.data = json.dumps({"stamp": round(now, 3), "fps": round(fps, 1),
                               "size": [w, h], "balls": balls})
        self.pub_det.publish(msg)

        if self.pub_img is not None:
            from sensor_msgs.msg import Image
            im = Image()
            im.height, im.width = h, w
            im.encoding = "bgr8"
            im.step = w * 3
            im.data = frame.tobytes()
            self.pub_img.publish(im)


def main():
    rclpy.init()
    node = BallDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
