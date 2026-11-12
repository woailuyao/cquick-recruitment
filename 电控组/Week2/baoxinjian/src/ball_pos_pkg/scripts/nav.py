#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pos_pkg.msg import BallPos
import math

BALLS = {1: (2.0, 3.0), 2: (5.5, 8.0), 3: (9.0, 4.0)}


class BallNav(Node):
    def __init__(self):
        super().__init__('ball_nav')
        self.pos_sub = self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.balls = {}
        self.targets = []
        self.current = 0
        self.pose = None
        self.started = False
        self.dbg = 0
        self.timer = self.create_timer(0.05, self.control_loop)

    def ball_cb(self, msg):
        if not self.started and msg.id not in self.balls:
            self.balls[msg.id] = (msg.x, msg.y)
            self.get_logger().info(f'got ball{msg.id}: ({msg.x}, {msg.y})')
            if len(self.balls) >= 3:
                self.targets = sorted(self.balls.items())
                self.started = True
                self.get_logger().info('3 balls ready, start nav!')

    def pose_cb(self, msg):
        self.pose = msg

    def control_loop(self):
        if not self.started or self.current >= len(self.targets) or self.pose is None:
            return
        tid, (tx, ty) = self.targets[self.current]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        angle_err = target_angle - self.pose.theta
        while angle_err > math.pi:
            angle_err -= 2 * math.pi
        while angle_err < -math.pi:
            angle_err += 2 * math.pi
        self.dbg += 1
        if self.dbg % 20 == 0:
            self.get_logger().info(
                f'>> ball{tid} target({tx},{ty}) pose({self.pose.x:.2f},{self.pose.y:.2f},th={self.pose.theta:.2f}) dist={dist:.2f} aerr={angle_err:.2f}')
        cmd = Twist()
        if dist < 0.15:
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'ball{tid} reached: ({self.pose.x:.2f}, {self.pose.y:.2f}) error {dist:.3f}')
            self.current += 1
            if self.current >= len(self.targets):
                self.get_logger().info('all balls done!')
            return
        if abs(angle_err) > 0.15:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-2.0, min(2.0, 2.0 * angle_err))
        else:
            cmd.linear.x = min(1.0, 0.4 * dist)
            cmd.angular.z = max(-2.0, min(2.0, 2.0 * angle_err))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BallNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


main()
