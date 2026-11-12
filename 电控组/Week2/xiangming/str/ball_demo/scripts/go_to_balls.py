#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ball_demo.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class GoToBalls(Node):
    def __init__(self):
        super().__init__('go_to_balls')
        self.balls = {}
        self.target_order = [1, 2, 3]
        self.target_idx = 0
        self.pose = None
        self.done = False

        self.sub_ball = self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control)

    def ball_cb(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)

    def pose_cb(self, msg):
        self.pose = msg

    def control(self):
        if self.pose is None or len(self.balls) < 3:
            return
        if self.target_idx >= len(self.target_order):
            if not self.done:
                self.done = True
                self.get_logger().info('三个球都到了')
                stop = Twist()
                self.pub_vel.publish(stop)
            return

        tid = self.target_order[self.target_idx]
        tx, ty = self.balls[tid]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)

        if dist < 0.5:
            self.get_logger().info(f'到达球{tid}，实际({self.pose.x:.2f},{self.pose.y:.2f})，误差{dist:.2f}')
            self.target_idx += 1
            return

        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.pose.theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        vel = Twist()
        if abs(angle_diff) > 0.1:
            vel.linear.x = 0.0
            vel.angular.z = 0.5 if angle_diff > 0 else -0.5
        else:
            vel.linear.x = min(0.5, dist * 0.3)
            vel.angular.z = 0.3 * angle_diff
        self.pub_vel.publish(vel)

def main():
    rclpy.init()
    node = GoToBalls()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
