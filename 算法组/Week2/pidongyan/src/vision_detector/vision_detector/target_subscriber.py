#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 2 周任务 3：订阅节点 —— 收 /target_position 并打印 detected/u/v/area。

对应细则 3.2：
    - 订阅 /target_position，打印 detected / u / v / area
    - 与发布节点的话题名、消息类型必须一致（不一致时 ROS2 会静默收不到，
      用 ros2 topic info /target_position 排查）

关键设计：回调里【必须先判断 detected，再使用 u/v】。
    这是细则 3.1 反复强调的语义：detected=False 时 u/v/area=0 只是"无意义"占位，
    如果不判断就把 (0,0) 当目标坐标，机器人会以为目标一直在左上角 —— 经典事故。

Ubuntu 运行：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run vision_detector target_subscriber

Windows 无 ROS 时自测（dry-run 会打印模拟收到的消息格式）：
    python target_subscriber.py --dry-run
"""
from __future__ import annotations

import argparse

# 与发布节点相同的容错导入：没有 ROS2 时仍可 dry-run
try:
    import rclpy
    from rclpy.node import Node
    from vision_interfaces.msg import TargetPosition
    ROS_OK = True
except ImportError:
    from dataclasses import dataclass

    @dataclass
    class TargetPosition:
        detected: bool = False
        u: float = 0.0
        v: float = 0.0
        area: float = 0.0

    Node = object
    rclpy = None
    ROS_OK = False


class TargetSubscriber(Node):
    """ROS2 订阅节点：收到一条消息打印一行，并统计条数。"""

    def __init__(self):
        super().__init__("target_subscriber")
        self.count = 0                       # 收到的总条数（验证"至少 10 条"）

        # create_subscription(消息类, 话题名, 回调, QoS)
        # 话题名和 QoS depth 必须与发布端一致，否则可能静默收不到
        self.sub = self.create_subscription(
            TargetPosition, "target_position", self.on_msg, 10)

        self.get_logger().info("等待 /target_position 消息 ...")

    def on_msg(self, msg: TargetPosition) -> None:
        """每收到一条消息触发一次。

        msg 的四个字段直接当属性读（.msg 定义了什么就有什么）。
        """
        self.count += 1

        if msg.detected:
            # 有效帧：放心使用 u/v/area
            self.get_logger().info(
                f"[{self.count:03d}] 检测到目标  u={msg.u:.1f}  v={msg.v:.1f}  "
                f"area={msg.area:.0f} px^2")
        else:
            # 无效帧：u/v/area 是占位 0，【不能】当成坐标使用
            self.get_logger().info(
                f"[{self.count:03d}] 未检测到目标（本帧无效，u/v/area 不使用）")


def _simulate(n: int = 12) -> None:
    """dry-run：模拟接收循环，演示两种帧的打印格式。"""
    print("[dry-run] 未连接 ROS2，以下是订阅端将打印的格式示例：")
    for i in range(n):
        if i % 5 == 4:      # 模拟一条无效帧
            print(f"[{i + 1:03d}] 未检测到目标（本帧无效，u/v/area 不使用）")
        else:
            u, v, a = 495.5 + i, 908.8 - i * 2, 290984 - i * 300
            print(f"[{i + 1:03d}] 检测到目标  u={u:.1f}  v={v:.1f}  area={a:.0f} px^2")


def main() -> int:
    parser = argparse.ArgumentParser(description="目标位置订阅节点（第 2 周任务 3）")
    parser.add_argument("--dry-run", action="store_true",
                        help="不启动 ROS，只打印消息格式示例")
    args = parser.parse_args()

    if args.dry_run or not ROS_OK:
        _simulate()
        return 0

    rclpy.init()
    node = TargetSubscriber()
    try:
        rclpy.spin(node)            # 订阅是被动等消息，spin 阻塞住直到 Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
