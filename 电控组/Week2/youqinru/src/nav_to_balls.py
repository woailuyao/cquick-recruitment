#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_publisher.msg import BallPos


class NavToBalls(Node):
    def __init__(self):
        super().__init__('nav_to_balls')
        self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.balls = {}
        self.pose = None
        self.order = [1, 2, 3]
        self.idx = 0
        self.started = False
        self.done = False
        self.create_timer(0.1, self.control)

    def ball_cb(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)

    def pose_cb(self, msg):
        self.pose = msg

    def control(self):
        if len(self.balls) < 3 or self.pose is None or self.done:
            return
        if not self.started:
            self.started = True
            self.get_logger().info('收齐 3 个球，开始导航')

        if self.idx >= len(self.order):
            self.stop()
            self.done = True
            self.get_logger().info('三个点全部到达，结束')
            return

        ball_id = self.order[self.idx]
        tx, ty = self.balls[ball_id]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)
        angle_to = math.atan2(dy, dx)
        err = math.atan2(math.sin(angle_to - self.pose.theta),
                         math.cos(angle_to - self.pose.theta))

        if dist < 0.25:
            self.stop()
            self.get_logger().info(
                f'到达球 {ball_id}，实际坐标 ({self.pose.x:.2f}, {self.pose.y:.2f})，误差 {dist:.2f}')
            self.idx += 1
            return

        cmd = Twist()
        if abs(err) > 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 if err > 0 else -1.5
        else:
            cmd.linear.x = min(1.2, dist * 0.8)
            cmd.angular.z = err * 2.0
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = NavToBalls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
