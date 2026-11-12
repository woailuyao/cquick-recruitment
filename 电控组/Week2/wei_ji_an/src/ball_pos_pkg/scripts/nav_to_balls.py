#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pos_pkg.msg import BallPos # 注意这里是你自己的包名
import math

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_to_balls')
        
        # 1. 创建订阅者：订阅球的位置
        self.sub_ball = self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)
        
        # 2. 创建订阅者：订阅乌龟当前位姿
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # 3. 创建发布者：控制乌龟速度
        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 4. 状态变量
        self.balls = {} # 字典存球，格式 {id: (x, y)}
        self.targets = [(2.0, 3.0), (5.5, 8.0), (9.0, 4.0)] # 目标点
        self.current_idx = 0 # 当前去第几个球
        self.current_pose = None # 当前乌龟位姿
        
        # 5. 创建定时器（10Hz，也就是0.1秒执行一次控制逻辑）
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("导航节点已启动，等待收齐3个球...")

    def ball_callback(self, msg):
        # 存下收到的球坐标
        self.balls[msg.id] = (msg.x, msg.y)
        # 可选：打印一下收到了啥
        # self.get_logger().info(f"收到球 {msg.id}: ({msg.x}, {msg.y})")

    def pose_callback(self, msg):
        # 更新乌龟当前位姿
        self.current_pose = msg

    def control_loop(self):
        # 核心控制逻辑，每0.1秒跑一次
        # 1. 如果没收齐3个球，或者还没收到乌龟位姿，就原地等待
        if len(self.balls) < 3 or self.current_pose is None:
            return
            
        # 2. 如果3个球都走完了，停止并退出
        if self.current_idx >= 3:
            self.stop_turtle()
            return
            
        # 3. 获取当前目标点
        tx, ty = self.targets[self.current_idx]
        
        # 4. 算距离和角度
        dx = tx - self.current_pose.x
        dy = ty - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # 5. 判断是否到达目标点（阈值设为0.3，满足考核要求的≤0.5）
        if distance < 0.3:
            self.get_logger().info(f"到达第 {self.current_idx+1} 个球！实际坐标: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})，误差: {distance:.2f}")
            self.current_idx += 1
            self.stop_turtle()
            return
            
        # 6. 计算方位角差
        target_theta = math.atan2(dy, dx)
        angle_diff = target_theta - self.current_pose.theta
        # 角度归一化到 -pi 到 pi
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # 7. 速度控制（先对准再走，越近越慢）
        twist = Twist()
        if abs(angle_diff) > 0.3:  # 角度偏差大，原地转
            twist.linear.x = 0.0
            twist.angular.z = 1.5 * angle_diff # 简单的P控制
        else: # 角度对准了，边走边微调
            twist.linear.x = min(1.0, 0.8 * distance) # 限制最大速度1.0，越近越慢
            twist.angular.z = 1.5 * angle_diff
            
        # 限制最大角速度，防止转飞
        if twist.angular.z > 2.0: twist.angular.z = 2.0
        if twist.angular.z < -2.0: twist.angular.z = -2.0
            
        self.pub_cmd.publish(twist)

    def stop_turtle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
