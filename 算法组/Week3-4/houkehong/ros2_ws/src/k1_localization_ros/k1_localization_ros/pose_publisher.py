# -*- coding: utf-8 -*-
"""
pose_publisher —— 不依赖相机与 Windows 侧的联调/演示节点。

按正弦轨迹发布一组合成位姿，用于验证话题链路、rviz2 显示与下游订阅者。
真实运行时请用 bridge_node 接收 Windows 侧的实拍定位结果。

  ros2 run k1_localization_ros pose_publisher --ros-args -p rate_hz:=10.0
"""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import String

TOPIC_POSE = "/localization/field_pose"
TOPIC_POSE2D = "/localization/field_pose_2d"


class PosePublisher(Node):
    def __init__(self):
        super().__init__("k1_synthetic_pose_publisher")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("radius", 1.5)
        self.rate = max(0.5, float(self.get_parameter("rate_hz").value))
        self.radius = float(self.get_parameter("radius").value)

        self.pub_pose = self.create_publisher(String, TOPIC_POSE, 10)
        self.pub_pose2d = self.create_publisher(Pose2D, TOPIC_POSE2D, 10)

        self.t = 0.0
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f"publishing synthetic pose at {self.rate} Hz")

    def tick(self):
        self.t += 1.0 / self.rate
        x = self.radius * math.cos(self.t)
        y = self.radius * math.sin(self.t)
        yaw = math.degrees(self.t) % 360.0

        st = {"valid": True, "status": "ok", "marker_count": 4, "inlier_count": 4,
              "residual": 0.05, "hypotheses": 6,
              "x_camera_map": x, "y_camera_map": y, "yaw_camera_map": yaw,
              "x_carrier_map": x, "y_carrier_map": y, "yaw_carrier_map": yaw}
        m = String()
        m.data = json.dumps(st, ensure_ascii=False)
        self.pub_pose.publish(m)

        p = Pose2D()
        p.x, p.y, p.theta = x, y, math.radians(yaw)
        self.pub_pose2d.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
