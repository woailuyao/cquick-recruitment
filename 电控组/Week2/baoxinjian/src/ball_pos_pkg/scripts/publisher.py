#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos
from rcl_interfaces.msg import SetParametersResult

BALLS = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.declare_parameter('period_sec', 0.5)
        period = self.get_parameter('period_sec').value
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(period, self.timer_callback)
        self.add_on_set_parameters_callback(self.on_param_change)
        self.get_logger().info('ball_publisher started, 2Hz publishing ball_pos')

    def on_param_change(self, params):
        for p in params:
            if p.name == 'period_sec' and p.value > 0:
                self.timer.cancel()
                self.timer = self.create_timer(p.value, self.timer_callback)
                self.get_logger().info(f'period changed to {p.value}s')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        for bid, x, y in BALLS:
            msg = BallPos()
            msg.id = bid
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'publish -> ball{msg.id}: x={msg.x}, y={msg.y}')


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


main()

