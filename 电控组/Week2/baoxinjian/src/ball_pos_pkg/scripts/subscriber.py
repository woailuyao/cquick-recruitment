#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        # 订阅同一话题 ball_pos，收到消息就触发 cb
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f'收到 <- 球{msg.id}: x={msg.x:.2f}, y={msg.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

