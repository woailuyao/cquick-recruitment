# -*- coding: utf-8 -*-
"""
ROS2 话题发布（细则 2.11）

**架构说明（重要）**

本机相机在 Windows、ROS2 在 WSL2，两者不共享设备也不共享内存。
所以视觉管线有两种跑法：

  方案 A（推荐，考试现场的标准做法）
      整条管线跑在有 ROS2 的机器上（原生 Linux / WSL2），
      直接 `run.py live --ros`，本模块原生发布。

  方案 B（本机现实：相机只在 Windows 侧可用）
      Windows 跑 `run.py live --status-port 5056` 把状态以 JSON 行广播出去，
      WSL 里跑 `scripts/ros_bridge_node.py`（见该脚本）接收并转发成 ROS2 话题。
      本模块的 RosBridge 在 WSL 里复用同一套字段。

发布的话题：

    /localization/field_pose        std_msgs/String   —— 完整字段 JSON
    /localization/field_pose_2d     geometry_msgs/Pose2D —— x/y/yaw(rad)，方便 rviz 直接画
    /soccer/ball                    std_msgs/String   —— 足球检测框 JSON

为什么用 String 装 JSON 而不是自定义 msg：
    自定义 msg 必须先建一个 ament 包并 colcon build，考核现场临时换机器很容易翻车。
    JSON 字段名与细则 2.11 的表格完全一致，评审用 `ros2 topic echo` 能直接看到全部字段。
    如果确实需要强类型消息，按 `docs/field_pose_msg.txt` 里的定义另建包即可。
"""

from __future__ import annotations

import json
import math

ROS_TOPIC_POSE = "/localization/field_pose"
ROS_TOPIC_POSE2D = "/localization/field_pose_2d"
ROS_TOPIC_BALL = "/soccer/ball"


class RosBridge:
    """rclpy 发布器。只能在装有 ROS2 且已 source 环境的环境里实例化。"""

    def __init__(self, node_name: str = "k1_field_localization",
                 frame_id: str = "map", qos_depth: int = 10):
        try:
            import rclpy
            from geometry_msgs.msg import Pose2D
            from std_msgs.msg import String
        except Exception as e:  # pragma: no cover - 只在没装 ROS2 的机器上触发
            raise RuntimeError(
                "rclpy 不可用，无法发布 ROS2 话题。\n"
                "  如果你在 Windows 侧运行：请改用 --status-port 由 WSL 里的\n"
                "  scripts/ros_bridge_node.py 转发（见 README 第 8 节）。\n"
                f"  原始错误: {e}"
            )

        self._rclpy = rclpy
        self._Pose2D = Pose2D
        self._String = String
        self.frame_id = frame_id

        rclpy.init(args=None)
        self.node = rclpy.create_node(node_name)
        self.pub_pose = self.node.create_publisher(String, ROS_TOPIC_POSE, qos_depth)
        self.pub_pose2d = self.node.create_publisher(Pose2D, ROS_TOPIC_POSE2D, qos_depth)
        self.pub_ball = self.node.create_publisher(String, ROS_TOPIC_BALL, qos_depth)
        self.node.get_logger().info(
            f"publishing {ROS_TOPIC_POSE}, {ROS_TOPIC_POSE2D}, {ROS_TOPIC_BALL}")

    # ------------------------------------------------------------ 发布
    def publish_pose(self, status: dict, stamp=None):
        """status 就是 Pipeline.process() 返回的 status 字典。"""
        msg = self._String()
        payload = dict(status)
        payload["frame_id"] = self.frame_id
        payload["stamp"] = float(stamp) if stamp is not None else self.node.get_clock().now().nanoseconds * 1e-9
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_pose.publish(msg)

        # 同步发一份强类型的 Pose2D，方便 rviz2 / 其他节点直接用
        if status.get("valid") and status.get("x_camera_map") is not None:
            p = self._Pose2D()
            p.x = float(status["x_camera_map"])
            p.y = float(status["y_camera_map"])
            p.theta = math.radians(float(status["yaw_camera_map"]))
            self.pub_pose2d.publish(p)

    def publish_ball(self, detections, stamp=None):
        """
        detections: [(label, conf, (x1,y1,x2,y2)), ...]，只发 label == 'ball' 的
        """
        balls = []
        for label, conf, box in detections:
            if label != "ball":
                continue
            x1, y1, x2, y2 = box
            balls.append({
                "confidence": round(float(conf), 4),
                "x1": round(float(x1), 1), "y1": round(float(y1), 1),
                "x2": round(float(x2), 1), "y2": round(float(y2), 1),
                "center_u": round((x1 + x2) / 2.0, 1),
                "center_v": round((y1 + y2) / 2.0, 1),
            })
        if not balls:
            return
        msg = self._String()
        msg.data = json.dumps({
            "stamp": float(stamp) if stamp is not None else self.node.get_clock().now().nanoseconds * 1e-9,
            "count": len(balls),
            "balls": balls,
        }, ensure_ascii=False)
        self.pub_ball.publish(msg)

    def spin_once(self):
        self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        try:
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass


class NullBridge:
    """没有 ROS2 时的占位实现，保证调用方代码不用写 if。"""

    def publish_pose(self, status, stamp=None):
        pass

    def publish_ball(self, detections, stamp=None):
        pass

    def spin_once(self):
        pass

    def shutdown(self):
        pass
