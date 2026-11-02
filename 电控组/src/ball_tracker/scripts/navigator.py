#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_tracker.msg import BallPos
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.balls = {}
        self.current_pose = None
        self.target_id = 1
        self.is_ready = False
        self.finished = False
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('导航节点已启动，等待接收3个球的位置...')

    def ball_callback(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        if len(self.balls) == 3 and not self.is_ready:
            self.is_ready = True
            self.get_logger().info('已收齐3个球，准备出发！')

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if not self.is_ready or self.current_pose is None or self.finished:
            return
            
        if self.target_id > 3:
            self.finished = True
            self.pub.publish(Twist()) 
            self.get_logger().info('任务完成！已到达所有目标点。')
            return

        target_x, target_y = self.balls[self.target_id]
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_pose.theta
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        cmd = Twist()
        
        if distance < 0.1:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            self.get_logger().info(f'到达球 {self.target_id}，实际坐标 ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})，误差 {distance:.2f}')
            self.target_id += 1 
            return

        if abs(angle_diff) > 0.2: 
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 if angle_diff > 0 else -0.8
        else:
            cmd.linear.x = min(1.5, distance * 2.0) 
            cmd.angular.z = angle_diff * 2.0

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
