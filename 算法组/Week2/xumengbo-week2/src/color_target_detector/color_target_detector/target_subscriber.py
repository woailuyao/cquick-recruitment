"""任务 3：订阅节点。

订阅 /target_position，逐条打印 detected / u / v / area。

运行：
    ros2 run color_target_detector target_subscriber
    ros2 run color_target_detector target_subscriber --ros-args -p topic:=/target_position
"""

import sys

import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition


class TargetSubscriber(Node):

    def __init__(self):
        super().__init__('target_subscriber')
        self.declare_parameter('topic', '/target_position')
        topic = self.get_parameter('topic').value

        self.count = 0
        self.sub = self.create_subscription(
            TargetPosition, topic, self.on_msg, 10)

        self.get_logger().info(
            f'订阅者已启动: topic={topic} type=target_interfaces/msg/TargetPosition')

    def on_msg(self, msg: TargetPosition):
        self.count += 1
        if msg.detected:
            print(f'[接收 #{self.count}] detected=True   '
                  f'u={msg.u:8.2f}  v={msg.v:8.2f}  area={msg.area:10.1f}', flush=True)
        else:
            # 用 detected 判断这一帧是否有效，而不是用 u/v 是否为 0 去猜
            print(f'[接收 #{self.count}] detected=False  未检测到目标 '
                  f'(u/v/area 为约定的 0)', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n共收到 {node.count} 条消息', flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
