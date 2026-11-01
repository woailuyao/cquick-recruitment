#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class BallNav(Node):
    def __init__(self):
        super().__init__('ball_nav')
        self.balls = {}
        self.targets = None
        self.pose = None
        self.i = 0
        self.state = 'WAIT'

        self.sub_ball = self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('导航节点启动：等待收齐 3 个球...')

    def ball_cb(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        if self.state == 'WAIT' and len(self.balls) >= 3:
            self.targets = [self.balls[1], self.balls[2], self.balls[3]]
            self.state = 'NAV'
            self.get_logger().info('球已收齐，开始导航：1 -> 2 -> 3')

    def pose_cb(self, msg):
        self.pose = msg

    def tick(self):
        tw = Twist()
        if self.state != 'NAV' or self.pose is None:
            return

        tx, ty = self.targets[self.i]
        dx, dy = tx - self.pose.x, ty - self.pose.y
        dist = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        err = math.atan2(math.sin(target_theta - self.pose.theta),
                         math.cos(target_theta - self.pose.theta))

        if dist < 0.15:
            self.get_logger().info(
                f'到达第 {self.i + 1} 个球！实际 ({self.pose.x:.2f}, {self.pose.y:.2f})，'
                f'目标 ({tx:.2f}, {ty:.2f})，误差 {dist:.3f}')
            self.i += 1
            if self.i >= 3:
                self.state = 'DONE'
                self.get_logger().info('三个球全部到达，任务完成')
            return

        if abs(err) > 0.15:
            tw.angular.z = max(-1.5, min(1.5, 2.0 * err))
        else:
            tw.linear.x = min(0.6, 0.8 * dist)
            tw.angular.z = max(-1.5, min(1.5, 2.0 * err))
        self.pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = BallNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
