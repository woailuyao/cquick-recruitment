#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_broadcast.msg import BallPos

class BallNavigator(Node):
    def __init__(self):
        super().__init__('ball_navigator')
        self.balls = {}
        self.id = 1
        self.wangba = None

        self.ball_sub = self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)
        self.wangba_sub = self.create_subscription(Pose, '/turtle1/pose', self.wangba_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def ball_callback(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)

    def wangba_callback(self, msg):
        self.wangba = msg

    def control_loop(self):
        if self.wangba is None or len(self.balls) < 3:
            return
        if self.id > 3:
            return

        tx, ty = self.balls[self.id]

        dx = tx - self.wangba.x
        dy = ty - self.wangba.y
        distance = math.sqrt(dx*dx + dy*dy)

        target_angle = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(target_angle - self.wangba.theta),
                                math.cos(target_angle - self.wangba.theta))

        twist = Twist()

        if distance < 0.2:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info(f'到达球 {self.id}，实际坐标 ({self.wangba.x:.2f}, {self.wangba.y:.2f})，误差 {distance:.2f}')
            self.id += 1
            return

        if abs(angle_diff) > 0.15:
            twist.linear.x = 0.0
            twist.angular.z = max(-2.5, min(2.5, 3.0 * angle_diff))
        else:
            twist.linear.x = max(0.05, min(0.7, distance * 1.2))
            twist.angular.z = max(-2.0, min(2.0, 2.5 * angle_diff))

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BallNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()









