# -*- coding: utf-8 -*-
"""
bridge_node —— 把 Windows 侧的定位/检测结果转发成 ROS2 话题。

为什么是"WSL 监听、Windows 连接"
--------------------------------
相机在 Windows、ROS2 在 WSL2，两者不共享内存与设备。
实测 WSL → Windows 的 TCP 会被防火墙/NAT 挡住，
而 Windows → WSL 的 localhost 端口转发是 WSL 原生支持的，所以方向反过来。

发布三个话题
------------
  /localization/field_pose     std_msgs/String   JSON，含细则要求全部字段
  /localization/field_pose_2d  geometry_msgs/Pose2D   x/y/theta（弧度）
  /soccer/ball                 std_msgs/String   足球检测 JSON

用法
----
  ros2 run k1_localization_ros bridge_node --ros-args -p port:=5056
  或
  ros2 launch k1_localization_ros localization.launch.py
"""
from __future__ import annotations

import json
import math
import socket

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import String

TOPIC_POSE = "/localization/field_pose"
TOPIC_POSE2D = "/localization/field_pose_2d"
TOPIC_BALL = "/soccer/ball"


class BridgeNode(Node):
    def __init__(self):
        super().__init__("k1_field_pose_relay")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5056)
        self.declare_parameter("qos_depth", 10)

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        depth = int(self.get_parameter("qos_depth").value)

        self.pub_pose = self.create_publisher(String, TOPIC_POSE, depth)
        self.pub_pose2d = self.create_publisher(Pose2D, TOPIC_POSE2D, depth)
        self.pub_ball = self.create_publisher(String, TOPIC_BALL, depth)

        self.n_pose = 0
        self.n_ball = 0
        self._buf = b""
        self._conn = None

        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(1)
        self.srv.settimeout(1.0)

        self.get_logger().info(f"publishing {TOPIC_POSE}, {TOPIC_POSE2D}, {TOPIC_BALL}")
        self.get_logger().info(f"listening {host}:{port}, waiting for the Windows side ...")

        self.create_timer(0.05, self.poll)

    # ---------------------------------------------------------------- 发布
    def publish_pose(self, st: dict):
        msg = String()
        msg.data = json.dumps(st, ensure_ascii=False)
        self.pub_pose.publish(msg)

        p2 = Pose2D()
        x = st.get("x_camera_map")
        y = st.get("y_camera_map")
        yaw = st.get("yaw_camera_map")
        if x is None or y is None or yaw is None or not st.get("valid"):
            return
        p2.x = float(x)
        p2.y = float(y)
        p2.theta = math.radians(float(yaw))
        self.pub_pose2d.publish(p2)
        self.n_pose += 1

    def publish_ball(self, balls):
        msg = String()
        msg.data = json.dumps({"balls": balls}, ensure_ascii=False)
        self.pub_ball.publish(msg)
        self.n_ball += 1

    # ---------------------------------------------------------------- 接收
    def poll(self):
        if self._conn is None:
            try:
                self._conn, addr = self.srv.accept()
                self._conn.settimeout(None)
                self.get_logger().info(f"connected: {addr}")
            except socket.timeout:
                return
            except OSError as e:
                self.get_logger().warn(f"accept error: {e}")
                return

        try:
            chunk = self._conn.recv(65536)
        except OSError as e:
            self.get_logger().warn(f"recv error: {e}")
            chunk = b""

        if not chunk:
            self.get_logger().info("peer closed, waiting for reconnect ...")
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None
            self._buf = b""
            return

        self._buf += chunk
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            if not line.strip():
                continue
            try:
                rec = json.loads(line.decode("utf-8"))
            except Exception:
                continue
            kind = rec.pop("_kind", "pose")
            if kind == "pose":
                self.publish_pose(rec)
            elif kind == "ball":
                self.publish_ball(rec.get("balls", []))

        total = self.n_pose + self.n_ball
        if total and total % 200 == 0:
            self.get_logger().info(f"pose={self.n_pose} ball={self.n_ball}")

    def shutdown_socket(self):
        try:
            if self._conn is not None:
                self._conn.close()
        except OSError:
            pass
        try:
            self.srv.close()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_socket()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
