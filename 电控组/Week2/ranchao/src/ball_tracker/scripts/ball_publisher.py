#!/usr/bin/env python3
"""任务1·发布者：按固定频率把 3 个球的坐标发到 /ball_pos。
频率默认 2 Hz，运行中可用 ros2 param set 调整（加分项）。"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from ball_tracker.msg import BallPos

# 三个球的坐标（细则 1.2 规定，不许改）
BALLS = [
    (1, 2.0, 3.0),   # 球1：左下
    (2, 5.5, 8.0),   # 球2：中上
    (3, 9.0, 4.0),   # 球3：右下
]


class BallPublisher(Node):

    def __init__(self):
        super().__init__('ball_publisher')
        # 发布者：话题名 ball_pos（跑起来就是 /ball_pos）
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)

        # 声明参数 rate（发布频率，默认 2 Hz），并允许运行中修改
        self.declare_parameter('rate', 2.0)
        self.add_on_set_parameters_callback(self.on_rate_change)
        self.timer = self.create_timer(1.0 / self.get_parameter('rate').value,
                                       self.publish_balls)

    def on_rate_change(self, params):
        """有人用 ros2 param set 改 rate 时触发：停旧定时器、按新频率建新的"""
        for p in params:
            if p.name == 'rate' and p.value > 0:
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / p.value, self.publish_balls)
                self.get_logger().info(f'发布频率已改为 {p.value} Hz')
        return SetParametersResult(successful=True)

    def publish_balls(self):
        """每个周期把 3 个球依次各发一条"""
        for ball_id, x, y in BALLS:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发布: id={msg.id} x={msg.x} y={msg.y}')


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
