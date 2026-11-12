#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from ball_comm.msg import BallPos

BALLS = [
    (1, 2.0, 3.0),
    (2, 5.5, 8.0),
    (3, 9.0, 4.0),
]


class BallPub(Node):
    def __init__(self):
        super().__init__('ball_pub')
        self.declare_parameter('pub_freq', 2.0)
        freq = self.get_parameter('pub_freq').value
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(1.0 / freq, self.timer_cb)
        self.add_on_set_parameters_callback(self.param_cb)
        self.get_logger().info(f'发布者启动，频率 {freq} Hz')

    def param_cb(self, params):
        for p in params:
            if p.name == 'pub_freq':
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / p.value, self.timer_cb)
                self.get_logger().info(f'频率改为 {p.value} Hz')
        return SetParametersResult(successful=True)

    def timer_cb(self):
        for bid, x, y in BALLS:
            msg = BallPos()
            msg.id = bid
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发布球 {msg.id}: x={msg.x:.2f}, y={msg.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = BallPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
