#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)

        self.declare_parameter('publish_rate', 2.0)
        self.add_on_set_parameters_callback(self._update_rate)

        self.balls = [
            {"id":1, "x":2.0, "y":3.0},
            {"id":2, "x":5.5, "y":8.0},
            {"id":3, "x":9.0, "y":4.0}
        ]
        self.index = 0
        # 初始定时器从参数读取，和默认值保持一致
        init_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / init_rate, self.timer_callback)

    def _update_rate(self, params):
        for p in params:
            if p.name == 'publish_rate':
                # 直接用本次传入的新值 p.value，不走节点存储，无延迟
                new_rate = max(p.value, 0.1)
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / new_rate, self.timer_callback)
                self.get_logger().info(f"发布频率已更新为 {new_rate:.1f} Hz")
        return rclpy.node.SetParametersResult(successful=True)

    def timer_callback(self):
        msg = BallPos()
        ball = self.balls[self.index]
        msg.id = ball["id"]
        msg.x = ball["x"]
        msg.y = ball["y"]
        self.publisher_.publish(msg)
        self.get_logger().info(f"发布球 {msg.id} x={msg.x} y={msg.y}")
        self.index = (self.index + 1) % 3

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

