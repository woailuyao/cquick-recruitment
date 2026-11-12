import rclpy
from rclpy.node import Node
from ball_interfaces.msg import BallPos

class BallPub(Node):
    def __init__(self):
        super().__init__('ball_pub')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        # 任务建议2Hz，周期0.5s
        self.timer = self.create_timer(0.5, self.timer_cb)
        # 三个球的列表
        self.ball_list = [
            {"id":1, "x":2.0, "y":3.0},
            {"id":2, "x":5.5, "y":8.0},
            {"id":3, "x":9.0, "y":4.0},
        ]

    def timer_cb(self):
        # 每一次定时器进来，循环把3个球全部发一遍
        for ball in self.ball_list:
            msg = BallPos()
            msg.id = ball["id"]
            msg.x = ball["x"]
            msg.y = ball["y"]
            self.pub.publish(msg)
            self.get_logger().info(f"发布球 id={msg.id}, x={msg.x}, y={msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = BallPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
