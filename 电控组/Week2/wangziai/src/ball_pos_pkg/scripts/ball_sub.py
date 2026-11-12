#!/usr/bin/env python3
"""订阅者：收到 BallPos 就打印 id / x / y。"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos

# 继承ROS2节点Node
class BallSubscriber(Node):
    def __init__(self):
        # 注册节点名
        super().__init__('ball_subscriber')
        # 创建订阅者（消息类型、话题名、订阅函数、QoS 队列深度）
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.on_msg, 10)

    # 接收函数
    def on_msg(self, msg):
        self.get_logger().info(f'已接收： 球id {msg.id}, 坐标 ({msg.x}, {msg.y})')

# 主程序
def main(args=None):
    # 脚本初始化
    rclpy.init(args=args)

    # 建立发布节点
    node = BallSubscriber()

    # 订阅节点接收内容
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    # 脚本安全结束
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
