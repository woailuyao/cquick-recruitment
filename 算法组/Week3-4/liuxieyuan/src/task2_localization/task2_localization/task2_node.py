#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CQuick 任务二 · 单目场地定位 ROS2 节点

两个话题：
    /task2/pose    geometry_msgs/Pose2D   x(m) y(m) theta(rad, 逆时针为正)
    /task2/status  std_msgs/String        valid=true marker_count=3 residual=0.090

两种数据源（参数 use_camera 切换）：
    use_camera:=false  仿真模式：用 field.yaml 里的标志点合成观测，绕场地画圆发布，
                       用于在没有相机 / 没有现场的机器上演示话题接口；
    use_camera:=true   实时模式：打开相机（或视频文件），用鼠标左键点标志与地面的接触点，
                       点击后自动解位姿并发布（与 task2_localization.py 逻辑一致）。

运行：
    ros2 run task2_localization task2_node --ros-args -p use_camera:=false
    ros2 launch task2_localization task2.launch.py
"""

import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import String

# 复用主程序的几何与匹配实现（同包内的 task2_localization.py）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from task2_localization import task2_localization as core     # 作为 ROS2 包安装时
except Exception:                                                  # 直接 python3 运行时
    import task2_localization as core


class Task2Node(Node):
    def __init__(self):
        super().__init__("task2_localization")
        self.declare_parameter("use_camera", False)
        self.declare_parameter("field_yaml", "config/field.yaml")
        self.declare_parameter("pitch_deg", core.PITCH_DEG)
        self.declare_parameter("height_m", core.CAM_HEIGHT)
        self.declare_parameter("residual_max", 0.20)
        self.declare_parameter("rate_hz", 10.0)

        self.residual_max = float(self.get_parameter("residual_max").value)
        self.markers = core.load_field(self.get_parameter("field_yaml").value)
        core.PITCH_DEG = float(self.get_parameter("pitch_deg").value)
        core.CAM_HEIGHT = float(self.get_parameter("height_m").value)
        self.get_logger().info(
            f"加载 {len(self.markers)} 个地图点；pitch={core.PITCH_DEG:.1f}° "
            f"h={core.CAM_HEIGHT:.2f} m residual_max={self.residual_max:.2f} m")

        self.pub_pose = self.create_publisher(Pose2D, "/task2/pose", 10)
        self.pub_status = self.create_publisher(String, "/task2/status", 10)

        self.use_camera = bool(self.get_parameter("use_camera").value)
        if self.use_camera:
            self._init_camera()
        else:
            self.t0 = self.get_clock().now()
            self.get_logger().info("仿真模式：绕场地画圆合成观测（use_camera:=true 可切实时）")

        rate = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.on_timer)

    # ---------------- 仿真数据源 ----------------
    def _sim_pose(self):
        """在场地里画一个半径 0.8 m 的圆，朝向随位置变化。"""
        t = self.get_clock().now().nanoseconds * 1e-9
        # 沿右侧边线附近缓慢移动，始终朝球门，模拟现场站位
        x = 0.30 + 0.45 * math.sin(t * 0.4)
        y = -2.05 + 0.15 * math.cos(t * 0.4)
        yaw = 8.0 * math.sin(t * 0.4)
        return x, y, yaw

    def _sim_observations(self, x, y, yaw):
        """把地图标志投影到相机系，模拟“看到 3 个标志”。"""
        c, s = math.cos(math.radians(-yaw)), math.sin(math.radians(-yaw))
        obs = []
        for (name, typ, mx, my) in self.markers:
            dx, dy = mx - x, my - y
            ox = c * dx - s * dy
            oy = s * dx + c * dy
            if ox < 0.5 or abs(math.degrees(math.atan2(oy, ox))) > 25.0:
                continue                      # 视野外
            obs.append((ox, oy))
            if len(obs) >= 3:
                break
        return obs

    # ---------------- 实时数据源 ----------------
    def _init_camera(self):
        import cv2
        self.cv2 = cv2
        src = 0
        self.cap = cv2.VideoCapture(src)
        self.obs = []
        if not self.cap.isOpened():
            self.get_logger().error("打不开相机，退回仿真模式")
            self.use_camera = False
            self.t0 = self.get_clock().now()
            return
        ok, frame = self.cap.read()
        if ok:
            core.set_frame_size(frame.shape[1], frame.shape[0])
        cv2.namedWindow("task2_localization")
        cv2.setMouseCallback("task2_localization", self._on_mouse)
        self.get_logger().info("左键点标志底部，c 清空，q 退出")

    def _on_mouse(self, event, x, y, flags, param):
        if event != 1:                        # EVENT_LBUTTONDOWN
            return
        X, Y = core.pixel_to_ground(x, y)
        if X is not None:
            self.obs.append((X, Y))
            self.get_logger().info(f"观测 {len(self.obs)}：像素({x},{y}) -> ({X:.2f},{Y:.2f})")

    # ---------------- 定时：解算 + 发布 ----------------
    def on_timer(self):
        if self.use_camera:
            ok, frame = self.cap.read()
            if not ok:
                return
            self.cv2.imshow("task2_localization", frame)
            if (self.cv2.waitKey(1) & 0xFF) == ord("q"):
                self.cv2.destroyAllWindows()
                self.cap.release()
                self.use_camera = False
                self.t0 = self.get_clock().now()
                return
            obs = self.obs
            pose = core.solve_pose_auto(obs, self.markers) if len(obs) >= 2 else None
        else:
            x, y, yaw = self._sim_pose()
            obs = self._sim_observations(x, y, yaw)
            pose = core.solve_pose_auto(obs, self.markers) if len(obs) >= 2 else None

        if pose is not None and pose[4] > self.residual_max:
            pose = None                       # 残差超限，按无效处理

        if pose is None:
            self.pub_status.publish(String(data=f"valid=false marker_count={len(obs)}"))
            return

        m = Pose2D()
        m.x = float(pose[0])
        m.y = float(pose[1])
        m.theta = float(math.radians(pose[2]))
        self.pub_pose.publish(m)
        self.pub_status.publish(String(
            data=f"valid=true marker_count={pose[3]} residual={pose[4]:.3f}"))
        self.get_logger().info(
            f"x={m.x:+.2f} y={m.y:+.2f} yaw={pose[2]:+.1f}°  "
            f"n={pose[3]} res={pose[4]*100:.0f} cm")


def main():
    rclpy.init()
    node = Task2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



