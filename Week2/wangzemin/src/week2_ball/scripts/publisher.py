#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from week2_ball.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        # 发布频率做成参数，默认 2 Hz，改频率不用动代码
        self.declare_parameter('freq', 2.0)
        freq = self.get_parameter('freq').value
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)
        self.add_on_set_parameters_callback(self.param_callback)
        self.get_logger().info(f'发布频率 = {freq} Hz')

    def param_callback(self, params):
        # 运行中 ros2 param set 也能立刻生效
        for p in params:
            if p.name == 'freq':
                if p.value <= 0.0:
                    return SetParametersResult(successful=False, reason='freq 必须大于 0')
                self.timer.cancel()
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / p.value, self.timer_callback)
                self.get_logger().info(f'发布频率已改成 {p.value} Hz')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]
        for b in balls:
            msg = BallPos()
            msg.id = b[0]
            msg.x = b[1]
            msg.y = b[2]
            self.publisher_.publish(msg)
            self.get_logger().info(f'发球: id={msg.id}, x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
