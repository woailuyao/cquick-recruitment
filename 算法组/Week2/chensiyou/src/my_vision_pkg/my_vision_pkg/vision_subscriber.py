#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.msg import TargetPosition

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber')
        # 订阅 /target_position 话题
        self.subscription = self.create_subscription(
            TargetPosition,
            '/target_position',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告

    def listener_callback(self, msg):
        # 打印接收到的消息，满足任务书要求的 detected / u / v / area
        self.get_logger().info(
            f'接收到消息 -> 检测到: {msg.detected}, u: {msg.u:.1f}, v: {msg.v:.1f}, 面积: {msg.area:.1f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VisionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()