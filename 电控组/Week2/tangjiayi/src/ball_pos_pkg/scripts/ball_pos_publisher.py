#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 1 · 发布端
每个周期把 3 个球的坐标依次发到话题 /ball_pos，同时打印自己发了什么。
频率可通过 ROS2 参数调整（不用改代码）：
    ros2 run ball_pos_pkg ball_pos_publisher --ros-args -p frequency:=5.0
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ball_pos_pkg.msg import BallPos

# 细则 1.2 里写死的三个坐标，一个都不能改
BALLS = [
    (1, 2.0, 3.0),  # 左下（我方半场）
    (2, 5.5, 8.0),  # 中上（对方半场）
    (3, 9.0, 4.0),  # 右下（边路）
]

TOPIC_NAME = "ball_pos"
QUEUE_SIZE = 10  # 队列长度：发得太快、对方来不及收时最多缓存 10 条


class BallPosPublisher(Node):
    def __init__(self):
        # 节点名，ros2 node list 里看到的就是它
        super().__init__("ball_pos_publisher")

        # 把频率做成参数，默认 2 Hz
        self.declare_parameter("frequency", 2.0)
        self.frequency = float(self.get_parameter("frequency").value)
        if self.frequency <= 0.0:
            self.get_logger().warn("frequency 必须大于 0，已回退为 2.0 Hz")
            self.frequency = 2.0

        # 创建发布者：消息类型 BallPos，话题名 ball_pos
        self.publisher = self.create_publisher(BallPos, TOPIC_NAME, QUEUE_SIZE)

        # 定时器：每 1/frequency 秒触发一次回调
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        self.get_logger().info(
            "发布端启动：话题=/%s，频率=%.1f Hz，每周期连发 %d 条"
            % (TOPIC_NAME, self.frequency, len(BALLS))
        )

    def timer_callback(self):
        # 一个周期里把 3 个球依次发出去（id 1 → 2 → 3）
        for ball_id, x, y in BALLS:
            msg = BallPos()
            msg.id = ball_id
            msg.x = float(x)
            msg.y = float(y)
            self.publisher.publish(msg)
            self.get_logger().info(
                "发布 -> id=%d  x=%.2f  y=%.2f" % (msg.id, msg.x, msg.y)
            )


def main(args=None):
    rclpy.init(args=args)
    node = BallPosPublisher()
    try:
        rclpy.spin(node)  # 阻塞在这里，等定时器不停触发
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 抛 KeyboardInterrupt；SIGTERM/kill 抛 ExternalShutdownException
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # 信号已经被 rclpy 自己关过一次时，不要重复 shutdown
            rclpy.shutdown()


if __name__ == "__main__":
    main()
