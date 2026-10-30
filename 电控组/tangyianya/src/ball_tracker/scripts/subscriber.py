import rclpy
from rclpy.node import Node
from ball_tracker.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.subscription = self.create_subscription(BallPos, 'ball_pos', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'收到球: ID={msg.id}, X={msg.x}, Y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BallSubscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
