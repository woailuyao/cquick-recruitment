#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pkg.msg import BallPos
import math
import time

class BallChaser(Node):
    def __init__(self):
        super().__init__('ball_chaser')
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.ball_sub = self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)

        self.current_pose = None
        self.balls = {}
        self.targets = []
        self.target_idx = 0
        
        # 状态机: 0=等待收集, 1=原地转向, 2=直线前进, 3=完成
        self.state = 0
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 速度常量
        self.linear_speed = 1.0
        self.angular_speed = 2.0
        self.start_time = 0

    def pose_callback(self, msg):
        self.current_pose = msg

    def ball_callback(self, msg):
        if msg.id not in self.balls:
            self.balls[msg.id] = msg
            self.get_logger().info(f'收集到球 {msg.id}')
        
        if len(self.balls) == 3 and self.state == 0:
            self.targets = [self.balls[1], self.balls[2], self.balls[3]]
            self.target_idx = 0
            self.state = 1
            self.start_time = time.time()
            self.get_logger().info('3个球已集齐，开始寻迹！')

    def control_loop(self):
        if self.state == 0 or self.current_pose is None:
            return
            
        cmd = Twist()
        target = self.targets[self.target_idx]
        
        # 计算到目标点的距离和角度
        dx = target.x - self.current_pose.x
        dy = target.y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        target_theta = math.atan2(dy, dx)
        
        angle_diff = target_theta - self.current_pose.theta
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi

        # 步骤 1: 原地转向对准目标
        if self.state == 1:
            if abs(angle_diff) > 0.05:
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed * (1 if angle_diff > 0 else -1)
            else:
                self.state = 2
                self.start_time = time.time()
                self.get_logger().info(f'已对准球 {target.id}，开始直线冲刺！')
                # 停止一下防止惯性
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return

        # 步骤 2: 直线前进，直到距离够近
        elif self.state == 2:
            if distance < 0.5:  # 误差小于 0.5 停车
                self.get_logger().info(f'到达球 {target.id}! 实际坐标: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}), 误差: {distance:.3f}')
                self.target_idx += 1
                if self.target_idx < len(self.targets):
                    self.state = 1  # 切到下一个球，重新转向
                    self.start_time = time.time()
                else:
                    self.get_logger().info('所有球均已到达，任务完成！')
                    self.state = 3
                    self.cmd_pub.publish(Twist()) # 停车
                    return
            else:
                # 持续前进
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BallChaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()