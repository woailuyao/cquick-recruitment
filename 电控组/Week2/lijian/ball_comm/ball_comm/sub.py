import rclpy
from rclpy.node import Node
from ball_interfaces.msg import BallPos

class BallSub(Node):
    def __init__(self):
        super().__init__('ball_sub')
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f"收到球 id={msg.id}, x={msg.x}, y={msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = BallSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
