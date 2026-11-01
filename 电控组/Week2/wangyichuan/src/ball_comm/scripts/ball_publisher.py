#!/usr/bin/env python3
"""任务 1 · 发布者：按固定频率循环发布 3 个球的坐标到话题 /ball_pos。

发布频率做成了 ROS 参数 publish_rate（默认 2 Hz），不用改代码就能调：
    ros2 param set /ball_publisher publish_rate 5.0
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ball_comm.msg import BallPos

# 三个球的坐标（考核细则给定的统一基准，不许改）
BALLS = [
    (1, 2.0, 3.0),  # 左下（我方半场）
    (2, 5.5, 8.0),  # 中上（对方半场）
    (3, 9.0, 4.0),  # 右下（边路）
]


class BallPublisher(Node):
    """每个周期把 id=1/2/3 三条 BallPos 依次发出去。"""

    def __init__(self):
        super().__init__('ball_publisher')

        self.declare_parameter('publish_rate', 2.0)

        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)

        # 用一个 100 Hz 的小定时器 + 时间累加来判断"该发了没"，
        # 这样 publish_rate 被 ros2 param set 改掉后立刻生效。
        self.last_publish_time = self.get_clock().now()
        self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(
            'ball_publisher 已启动，话题 /ball_pos，频率 %.1f Hz'
            % self.get_parameter('publish_rate').value
        )

    def timer_callback(self):
        rate = self.get_parameter('publish_rate').value
        if rate <= 0.0:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_publish_time).nanoseconds / 1e9
        if elapsed < 1.0 / rate:
            return
        self.last_publish_time = now

        # 一个周期把 3 个球依次发一遍
        for ball_id, x, y in BALLS:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.publisher_.publish(msg)
            self.get_logger().info(
                '发布 BallPos: id=%d x=%.2f y=%.2f' % (msg.id, msg.x, msg.y)
            )


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
