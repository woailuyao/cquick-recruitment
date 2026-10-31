#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from week2_ball.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')
        
        # 1. 发布速度指令
        self.pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 2. 订阅球的位置
        self.sub_ball = self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)
        
        # 3. 订阅乌龟当前的位置
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.balls = {}
        self.current_target = None
        self.target_id = 1 
        self.arrived = False

    def ball_callback(self, msg):
        # 用字典存球，收齐3个再动
        if len(self.balls) < 3:
            self.balls[msg.id] = (msg.x, msg.y)
            self.get_logger().info(f'已收集到球 {msg.id} 坐标: ({msg.x}, {msg.y})')
            if len(self.balls) == 3 and self.current_target is None:
                self.get_logger().info('3个球坐标已收齐! 开始导航...')
                self.set_next_target()

    def set_next_target(self):
        if self.target_id <= 3:
            self.current_target = self.balls[self.target_id]
            self.get_logger().info(f'目标: 前往球 {self.target_id}, 坐标 {self.current_target}')
            self.arrived = False
        else:
            self.get_logger().info('所有球已依次到达! 任务完成。')
            self.current_target = None
            self.stop_robot()

    def pose_callback(self, msg):
        if self.current_target is None or self.arrived:
            return

        gx, gy = self.current_target
        x, y, theta = msg.x, msg.y, msg.theta

        # 核心逻辑 1：算距离
        dist = math.sqrt((gx - x)**2 + (gy - y)**2)

        # 到达判定 (误差 <= 0.5，我们设 0.3 更精准)
        if dist < 0.3:
            self.get_logger().info(f'到达球 {self.target_id}! 实际坐标 ({x:.2f}, {y:.2f}), 误差 {dist:.2f}')
            self.arrived = True
            self.stop_robot()
            self.target_id += 1
            self.set_next_target()
            return

        # 核心逻辑 2：算方位角
        target_angle = math.atan2(gy - y, gx - x)
        angle_diff = target_angle - theta
        # 归一化到 [-pi, pi]
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        cmd = Twist()
        # 先对准再走
        if abs(angle_diff) > 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 * angle_diff # 原地转
        else:
            # 边走边微调，越近越慢 (防冲过头)
            cmd.linear.x = min(1.5 * dist, 1.0)
            cmd.angular.z = 2.0 * angle_diff
        
        self.pub_vel.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
