#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CQuick 任务二 · ROS2 接口演示节点（零依赖版）

用途：在**没有相机、没有 OpenCV** 的机器（例如虚拟机）上演示话题接口。
只依赖 rclpy + math，不 import cv2 / numpy。

发布：
    /task2/pose     geometry_msgs/Pose2D   x(m) y(m) theta(rad)
    /task2/status   std_msgs/String        valid=true marker_count=3 residual=0.0xx

说明：位姿由内置运动模型生成，用于验证“话题、消息、频率、状态字段”是否符合接口规范；
      真实定位请用 task2_node（需要相机/视频源与 OpenCV）。
"""

import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import String


class SimPublisher(Node):
    def __init__(self):
        super().__init__("task2_localization_sim")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("center_x", 0.30)
        self.declare_parameter("center_y", -2.05)
        self.declare_parameter("radius_x", 0.45)
        self.declare_parameter("radius_y", 0.15)
        self.declare_parameter("omega", 0.40)

        self.rate = float(self.get_parameter("rate_hz").value)
        self.cx = float(self.get_parameter("center_x").value)
        self.cy = float(self.get_parameter("center_y").value)
        self.rx = float(self.get_parameter("radius_x").value)
        self.ry = float(self.get_parameter("radius_y").value)
        self.w = float(self.get_parameter("omega").value)

        self.pub_pose = self.create_publisher(Pose2D, "/task2/pose", 10)
        self.pub_status = self.create_publisher(String, "/task2/status", 10)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(1.0 / max(self.rate, 1.0), self.on_timer)
        self.get_logger().info(
            f"接口演示模式：{self.rate:.0f} Hz，发布 /task2/pose 与 /task2/status")

    def on_timer(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        x = self.cx + self.rx * math.sin(t * self.w)
        y = self.cy + self.ry * math.cos(t * self.w)
        yaw_deg = 8.0 * math.sin(t * self.w)

        m = Pose2D()
        m.x = float(x)
        m.y = float(y)
        m.theta = math.radians(yaw_deg)
        self.pub_pose.publish(m)

        residual = 0.05 + 0.03 * abs(math.sin(t * 0.7))
        self.pub_status.publish(String(
            data=f"valid=true marker_count=3 residual={residual:.3f}"))
        self.get_logger().info(
            f"x={x:+.2f} y={y:+.2f} yaw={yaw_deg:+.1f}°  n=3 res={residual*100:.0f} cm")


def main():
    rclpy.init()
    node = SimPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
