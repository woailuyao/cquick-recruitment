#!/usr/bin/env python3
# 任务1 · 发布者
# 每 0.5 秒（2 Hz）把 3 个球的坐标依次发一遍到话题 ball_pos

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from ball_pos.msg import BallPos

# 细则规定的三个球，坐标写死，一个都不能改
BALLS = [
    (1, 2.0, 3.0),   # 左下（我方半场）
    (2, 5.5, 8.0),   # 中上（对方半场）
    (3, 9.0, 4.0),   # 右下（边路）
]


class BallPublisher(Node):

    def __init__(self):
        super().__init__('ball_publisher')

        # 发布频率做成参数，可以用 ros2 param set 在线改，不用改代码重编译（加分项）
        self.declare_parameter('freq', 2.0)

        # 创建发布者：消息类型 BallPos，话题名 ball_pos，队列长度 10
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)

        self.index = 0
        self.timer = None
        self._make_timer(self.get_parameter('freq').value)

        self.add_on_set_parameters_callback(self.on_param)

    def _make_timer(self, freq):
        """按频率重建定时器"""
        if self.timer is not None:
            self.destroy_timer(self.timer)
        self.timer = self.create_timer(1.0 / freq, self.tick)
        self.get_logger().info(
            f'发布频率 = {freq:.2f} Hz，周期 {1.0 / freq:.3f} s')

    def on_param(self, params):
        """支持 ros2 param set /ball_publisher freq 5.0 实时改频率"""
        for p in params:
            if p.name == 'freq':
                if p.value <= 0.0:
                    return SetParametersResult(successful=False)
                self.get_logger().info(f'频率被改为 {p.value:.2f} Hz')
                self._make_timer(p.value)
        return SetParametersResult(successful=True)

    def tick(self):
        """定时器每触发一次，就发一个球；三个球轮流发"""
        ball_id, x, y = BALLS[self.index % 3]

        msg = BallPos()
        msg.id = ball_id
        msg.x = x
        msg.y = y

        self.pub.publish(msg)
        self.get_logger().info(
            f'发布 球{msg.id}: x={msg.x:.2f} y={msg.y:.2f}')

        self.index += 1


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
