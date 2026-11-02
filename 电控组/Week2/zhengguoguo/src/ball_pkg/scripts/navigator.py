#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pkg.msg import BallPos
import math

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)

        self.pose = None
        self.balls = {}         
        self.target_idx = 1
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.started = False

        self.get_logger().info('导航节点已启动，等待球坐标...')

    def ball_callback(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        
        if len(self.balls) == 3 and not self.started and self.pose is not None:
            self.started = True
            self.get_logger().info(f'收齐 3 个球！准备前往 1 号球 ({self.balls[1][0]}, {self.balls[1][1]})')
            self.set_target(1)

    def set_target(self, idx):
        self.target_idx = idx
        self.goal_x, self.goal_y = self.balls[idx]

    def pose_callback(self, msg):
        self.pose = msg
        if not self.started:
            return
        
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.pose.theta
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        cmd = Twist()

        if distance < 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            
            self.get_logger().info(f'到达第 {self.target_idx} 个球，实际坐标 ({self.pose.x:.2f}, {self.pose.y:.2f})，误差 {distance:.2f}')
            
            if self.target_idx < 3:
                next_idx = self.target_idx + 1
                self.set_target(next_idx)
                self.get_logger().info(f'准备前往 {next_idx} 号球 ({self.goal_x}, {self.goal_y})')
            else:
                self.get_logger().info('全部 3 个球已巡游完毕')
                self.started = False
            return

        if abs(angle_diff) > 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 2.0 * angle_diff
        else:
            cmd.linear.x = 1.0 * distance
            cmd.angular.z = 2.0 * angle_diff
            
            if cmd.linear.x > 1.5:
                cmd.linear.x = 1.5

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = TurtleNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
