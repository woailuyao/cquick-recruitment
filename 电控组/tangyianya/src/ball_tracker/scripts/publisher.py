import rclpy
from rclpy.node import Node
from ball_tracker.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.balls = [{'id': 1, 'x': 2.0, 'y': 3.0}, {'id': 2, 'x': 5.5, 'y': 8.0}, {'id': 3, 'x': 9.0, 'y': 4.0}]

    def timer_callback(self):
        for ball in self.balls:
            msg = BallPos()
            msg.id = ball['id']
            msg.x = ball['x']
            msg.y = ball['y']
            self.publisher_.publish(msg)
            self.get_logger().info(f'发布球: ID={msg.id}, X={msg.x}, Y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BallPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
