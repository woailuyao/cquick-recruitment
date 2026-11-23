#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_task.msg import BallPos


class NavBalls(Node):
    def __init__(self):
        super().__init__('nav_balls')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.timer = self.create_timer(0.1, self.control)

        self.balls = {}
        self.targets = []
        self.idx = 0
        self.pose = None

    def ball_cb(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        if len(self.balls) == 3 and not self.targets:
            self.targets = [self.balls[i] for i in sorted(self.balls)]
            self.get_logger().info(f'收齐 3 颗球: {self.targets}')

    def pose_cb(self, msg):
        self.pose = msg

    def control(self):
        if not self.targets or self.pose is None:
            return
        if self.idx >= len(self.targets):
            self.pub.publish(Twist())
            return

        tx, ty = self.targets[self.idx]
        x, y, theta = self.pose.x, self.pose.y, self.pose.theta
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)

        if dist < 0.3:
            self.get_logger().info(
                f'到达第 {self.idx + 1} 个球，实际坐标 ({x:.2f}, {y:.2f})，误差 {dist:.2f}')
            self.pub.publish(Twist())
            self.idx += 1
            return

        err = math.atan2(dy, dx) - theta
        err = math.atan2(math.sin(err), math.cos(err))

        cmd = Twist()
        if abs(err) > 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-2.0, min(2.0, 1.5 * err))
        else:
            cmd.linear.x = min(0.6, 0.8 * dist)
            cmd.angular.z = max(-2.0, min(2.0, 1.5 * err))
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavBalls()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
