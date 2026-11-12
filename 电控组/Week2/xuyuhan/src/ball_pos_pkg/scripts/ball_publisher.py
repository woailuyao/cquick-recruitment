#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from ball_pos_pkg.msg import BallPos

# 三个球的坐标：写死，一个都不能改
BALLS = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')

        # 频率做成参数，运行中可以 ros2 param set 改（加分项 +3）
        self.declare_parameter('freq', 2.0)
        freq = self.get_parameter('freq').value

        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(1.0 / freq, self.on_timer)
        self.add_on_set_parameters_callback(self.on_param)
        self.get_logger().info(f'ball_publisher 启动，频率 {freq} Hz')

    def on_param(self, params):
        """频率被 ros2 param set 修改时，真正改掉定时器周期"""
        for p in params:
            if p.name == 'freq' and p.value > 0:
                self.timer.timer_period_ns = int(1e9 / p.value)
                self.get_logger().info(f'频率改为 {p.value} Hz')
        return SetParametersResult(successful=True)

    def on_timer(self):
        """每个周期把 3 个球依次发一遍"""
        for bid, x, y in BALLS:
            msg = BallPos()
            msg.id = bid
            msg.x = x
            msg.y = y
            self.publisher_.publish(msg)
            self.get_logger().info(f'发出 球{msg.id}: ({msg.x:.2f}, {msg.y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()