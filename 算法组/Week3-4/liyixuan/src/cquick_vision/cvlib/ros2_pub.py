"""ROS2 发布（可选）。rclpy 缺失时不影响主程序运行。

话题：
  /localization/field_pose      geometry_msgs/Pose2D   x,y 为相机光心地图坐标(m)，theta 为弧度
  /localization/field_pose_json std_msgs/String       完整字段（含 valid/status/residual，deg）
  /soccer/ball                  std_msgs/Float32MultiArray
                                [conf, x1, y1, x2, y2, center_u, center_v] × N

若已编译 cquick_interfaces（见 msg/FieldPose.msg），可直接替换为自定义消息。
"""
from __future__ import annotations

import json
import math
import threading
from typing import Optional, Sequence


class Ros2Bridge:
    def __init__(self, node_name: str = "cquick_vision", frame_id: str = "map"):
        import rclpy
        from geometry_msgs.msg import Pose2D
        from std_msgs.msg import Float32MultiArray, String

        self._rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node(node_name)
        self.frame_id = frame_id
        self.pub_pose = self.node.create_publisher(Pose2D, "/localization/field_pose", 10)
        self.pub_json = self.node.create_publisher(String, "/localization/field_pose_json", 10)
        self.pub_ball = self.node.create_publisher(Float32MultiArray, "/soccer/ball", 10)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self.node.get_logger().info("cquick_vision ROS2 bridge up")

    def _spin(self) -> None:
        while not self._stop.is_set():
            try:
                self._rclpy.spin_once(self.node, timeout_sec=0.05)
            except Exception:
                break

    def publish_pose(self, cam_pose_rad, valid: bool, marker_count: int,
                     residual: float, status: str, carrier_pose_rad=None) -> None:
        from geometry_msgs.msg import Pose2D
        from std_msgs.msg import String

        x, y, th = cam_pose_rad
        m = Pose2D()
        m.x, m.y, m.theta = float(x), float(y), float(th)
        self.pub_pose.publish(m)

        payload = {
            "stamp": self.node.get_clock().now().nanoseconds / 1e9,
            "frame_id": self.frame_id,
            "x_camera_map": float(x),
            "y_camera_map": float(y),
            "yaw_camera_map": math.degrees(th),
            "valid": bool(valid),
            "marker_count": int(marker_count),
            "residual": float(residual) if residual == residual else None,
            "status": str(status),
        }
        if carrier_pose_rad is not None:
            cx, cy, cth = carrier_pose_rad
            payload.update({
                "x_carrier_map": float(cx),
                "y_carrier_map": float(cy),
                "yaw_carrier_map": math.degrees(cth),
            })
        self.pub_json.publish(String(data=json.dumps(payload)))

    def publish_balls(self, dets: Sequence) -> None:
        from std_msgs.msg import Float32MultiArray
        arr = Float32MultiArray()
        data = []
        for d in dets:
            u, v = d.center
            data += [float(d.conf), float(d.x1), float(d.y1), float(d.x2), float(d.y2),
                     float(u), float(v)]
        arr.data = data
        self.pub_ball.publish(arr)

    def shutdown(self) -> None:
        self._stop.set()
        try:
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass
