#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_publisher.msg import BallPos

BALLS = [
    (1, 2.0, 3.0),
    (2, 5.5, 8.0),
    (3, 9.0, 4.0),
]

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_pub_node')
        self.declare_parameter('frequency', 2.0)
        freq = self.get_parameter('frequency').value
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)

    def timer_callback(self):
        for ball_id, x, y in BALLS:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发送 id={ball_id} x={x} y={y}')

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

if __name__ == '__main__':
    main()
