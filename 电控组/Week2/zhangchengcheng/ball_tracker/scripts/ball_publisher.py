#!/usr/bin/env python3
"""任务 1 · 发布者

把细则规定的 3 个球坐标，按固定频率依次发布到话题 /ball_pos。

频率用 ROS 2 参数控制，不用改代码：
    ros2 run ball_tracker ball_publisher.py --ros-args -p publish_frequency:=5.0
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ball_tracker.msg import BallPos

# 细则 1.2 规定的三个球坐标，不允许修改
BALLS = [
    (1, 2.0, 3.0),   # 左下（我方半场）
    (2, 5.5, 8.0),   # 中上（对方半场）
    (3, 9.0, 4.0),   # 右下（边路）
]


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.declare_parameter('publish_frequency', 2.0)
        self.declare_parameter('topic_name', 'ball_pos')

        freq = float(self.get_parameter('publish_frequency').value)
        topic = self.get_parameter('topic_name').value

        self.publisher_ = self.create_publisher(BallPos, topic, 10)
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)
        self.get_logger().info(
            f'发布者已启动：话题 /{topic}，频率 {freq} Hz，每周期发 3 条')

    def timer_callback(self):
        for ball_id, x, y in BALLS:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.publisher_.publish(msg)
            self.get_logger().info(
                f'发布  球{msg.id}  x={msg.x:.2f}  y={msg.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
