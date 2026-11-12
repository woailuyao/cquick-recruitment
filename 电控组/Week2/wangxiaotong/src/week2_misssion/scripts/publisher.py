#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from week2_mission1.msg import BallPos
# ⚠️ 新增：导入参数设置的结果类型
from rcl_interfaces.msg import SetParametersResult

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        
        self.balls = [
            (1, 2.0, 3.0),
            (2, 5.5, 8.0),
            (3, 9.0, 4.0)
        ]
        
        # ⚠️ 新增：1. 声明参数 'freq'，默认值 2.0
        self.declare_parameter('freq', 2.0)
        
        # ⚠️ 修改：2. 从参数中读取当前的频率，并计算周期
        current_freq = self.get_parameter('freq').value
        self.timer = self.create_timer(1.0 / current_freq, self.timer_callback)

        # ⚠️ 新增：3. 注册参数变化的回调函数，当有人动态修改参数时会触发
        self.add_on_set_parameters_callback(self.parameter_callback)

    # ⚠️ 新增：这是一个专门处理参数变化的方法
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'freq':
                self.get_logger().info(f'检测到频率更新：{param.value} Hz')
                # 销毁原来的定时器
                self.destroy_timer(self.timer)
                # 用新的频率创建新的定时器
                self.timer = self.create_timer(1.0 / param.value, self.timer_callback)
        return SetParametersResult(successful=True)

    def timer_callback(self):
        for ball_id, x, y in self.balls:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.publisher_.publish(msg)
            self.get_logger().info(f'发布球 {msg.id}: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()