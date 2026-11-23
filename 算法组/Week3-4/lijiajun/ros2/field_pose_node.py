"""VM 端桥接节点：UDP 收 Windows 推理结果(JSON) → 发布 /localization/field_pose。

前置（VM，ROS2 环境）：
  1) 用 colcon 建一个消息包（如 field_pose_msgs），把 msg/FieldPose.msg 编进去；
  2) source install/setup.bash；
  3) python field_pose_node.py --port 9000
Windows 端（宿主，跑推理）：
  python src/p3p.py 1 --send <VM_IP>:9000

注意：本机没有 ROS2，此文件未经运行验证，仅按细则 2.11 的字段组织。
"""
import argparse
import json
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from field_pose_msgs.msg import FieldPose, Ball


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--topic", default="/localization/field_pose")
    ap.add_argument("--frame-id", default="field")
    a = ap.parse_args()

    rclpy.init()
    node = Node("field_pose_bridge")
    pub = node.create_publisher(FieldPose, a.topic, 10)
    ball_pub = node.create_publisher(Ball, "/soccer/ball", 10)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", a.port))
    sock.settimeout(0.2)
    node.get_logger().info(f"UDP listen 0.0.0.0:{a.port} -> {a.topic}")
    while rclpy.ok():
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            rclpy.spin_once(node, timeout_sec=0.0)
            continue
        d = json.loads(data.decode("utf-8"))
        m = FieldPose()
        m.header = Header()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = a.frame_id
        m.x_camera_map = float(d.get("x_camera_map", 0.0))
        m.y_camera_map = float(d.get("y_camera_map", 0.0))
        m.yaw_camera_map = float(d.get("yaw_camera_map", 0.0))
        m.x_carrier_map = float(d.get("x_camera_map", 0.0))
        m.y_carrier_map = float(d.get("y_camera_map", 0.0))
        m.yaw_carrier_map = float(d.get("yaw_camera_map", 0.0))
        m.valid = bool(d.get("valid", False))
        m.marker_count = int(d.get("marker_count", 0))
        m.supplement_added = int(d.get("supplement_added", 0))
        m.residual = float(d.get("residual", 0.0))
        m.status = str(d.get("status", ""))
        pub.publish(m)
        bd = d.get("ball")
        bm = Ball()
        bm.header = m.header
        bm.visible = bd is not None
        if bd:
            bm.u = float(bd.get("u", 0.0))
            bm.v = float(bd.get("v", 0.0))
            bm.conf = float(bd.get("conf", 0.0))
        ball_pub.publish(bm)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
