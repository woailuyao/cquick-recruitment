# -*- coding: utf-8 -*-
"""ROS2 发布器（可选）。只用标准消息，不需要先编译自定义 msg。

话题：
    /soccer/ball                       std_msgs/Float32MultiArray
                                      data = [count, u1, v1, conf1, u2, v2, conf2, ...]
                                      u/v 是足球框中心像素，conf 是置信度
    /localization/field_pose           geometry_msgs/Pose2D
                                      x = x_camera_map [m], y = y_camera_map [m],
                                      theta = yaw_camera_map [rad]
    /localization/field_pose_info      std_msgs/String（JSON）
                                      {"valid":..,"marker_count":..,"residual":..,"status":..,
                                       "yaw_deg":..,"fps":..,"balls":..}

如果本机没装 ROS2，create_publisher() 返回 None，调用方继续正常显示界面。
"""
from __future__ import annotations

import json


class Ros2Publisher:
    def __init__(self, node_name: str = "week34_vision"):
        import rclpy
        from geometry_msgs.msg import Pose2D
        from std_msgs.msg import Float32MultiArray, String

        self._rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node(node_name)
        self.pub_ball = self.node.create_publisher(Float32MultiArray, "/soccer/ball", 10)
        self.pub_pose = self.node.create_publisher(Pose2D, "/localization/field_pose", 10)
        self.pub_info = self.node.create_publisher(String, "/localization/field_pose_info", 10)
        self._Pose2D = Pose2D
        self._F32 = Float32MultiArray
        self._String = String
        self.node.get_logger().info(
            "发布: /soccer/ball  /localization/field_pose  /localization/field_pose_info")

    def publish(self, balls, pose, res, fps):
        """balls: [(u, v, conf), ...]；pose: {x,y,yaw_deg} 或 None；res: solve 结果 dict"""
        # 足球
        data = [float(len(balls))]
        for (u, v, c) in balls:
            data += [float(u), float(v), float(c)]
        m = self._F32()
        m.data = data
        self.pub_ball.publish(m)

        # 位姿（Pose2D: x, y, theta[rad]）
        p = self._Pose2D()
        if pose is not None:
            p.x = float(pose["x"])
            p.y = float(pose["y"])
            p.theta = float(pose["yaw_deg"]) * 3.141592653589793 / 180.0
        self.pub_pose.publish(p)

        # 状态信息（JSON 字符串）
        info = {
            "valid": bool(res.get("valid", False)),
            "marker_count": int(res.get("marker_count", 0)),
            "residual": float(res["residual"]) if res.get("residual") is not None else -1.0,
            "status": str(res.get("status", "lost")),
            "yaw_deg": float(pose["yaw_deg"]) if pose else None,
            "fps": round(float(fps), 1),
            "balls": len(balls),
        }
        s = self._String()
        s.data = json.dumps(info, ensure_ascii=False)
        self.pub_info.publish(s)

        # 让发布真的发出去（不 spin 也能 flush，但保险起见转一下）
        self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        try:
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass


def create_publisher_or_none(node_name: str = "week34_vision"):
    """装了 ROS2 就返回发布器，没装返回 None（不抛异常）。"""
    try:
        return Ros2Publisher(node_name)
    except Exception as e:  # rclpy 没装 / 环境变量没 source
        print(f"[WARN] ROS2 不可用，跳过话题发布：{e}")
        print("       装好 ROS2 后加 --ros2 参数即可同时发布话题。")
        return None
