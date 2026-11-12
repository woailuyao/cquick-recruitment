#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from new_bag.msg import BallPos


class BallNavigator(Node):
    def __init__(self):
        super().__init__('ball_navigator')
        self.balls = {}
        self.targets = []
        self.current_idx = 0
        self.pose = None
        self.done = False
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

    def ball_cb(self, msg):
        if self.done:
            return
        self.balls[msg.id] = (msg.x, msg.y)
        if len(self.balls) == 3 and len(self.targets) == 0:
            self.targets = [self.balls[i] for i in (1, 2, 3)]


    def pose_cb(self, msg):
        self.pose = msg
        self.control()

    def control(self):
        if self.pose is None or not self.targets or self.done:
            return
        if self.current_idx >= len(self.targets):
            self.done = True
            self.pub.publish(Twist())
            self.get_logger().info('全部目标到达')
            return

        tx, ty = self.targets[self.current_idx]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)

        if dist < 0.3:
            self.pub.publish(Twist())
            self.get_logger().info(
                f'到达球 {self.current_idx + 1}，'
                f'实际坐标 ({self.pose.x:.3f}, {self.pose.y:.3f})，误差 {dist:.3f}'
            )
            self.current_idx += 1
            return

        target_angle = math.atan2(dy, dx)
        diff = self.normalize_angle(target_angle - self.pose.theta)

        cmd = Twist()
        if abs(diff) > 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = max(min(2.0 * diff, 2.0), -2.0)
        else:
            cmd.linear.x = min(1.5 * dist, 1.0)
            cmd.angular.z = max(min(2.0 * diff, 1.5), -1.5)
        self.pub.publish(cmd)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = BallNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
