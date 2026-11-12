#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleNav(Node):
    def __init__(self):
        super().__init__('turtle_nav')
        # 发布速度
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 订阅位置
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.pose = None
        # 考核规定的三个点
        self.targets = [(2.0, 3.0), (5.5, 8.0), (9.0, 4.0)]
        self.current_target_index = 0
        
        # 控制频率：10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        # 如果还没收到位置信息，或者三个点都走完了，就停下
        if self.pose is None or self.current_target_index >= len(self.targets):
            return
            
        target_x, target_y = self.targets[self.current_target_index]
        current_x = self.pose.x
        current_y = self.pose.y
        current_theta = self.pose.theta

        # 1. 计算到目标点的距离
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # 2. 判断是否到达（误差 < 0.1）
        if distance < 0.1:
            self.get_logger().info(f'--- 到达球 {self.current_target_index + 1}！实际坐标:({current_x:.2f}, {current_y:.2f})，误差:{distance:.2f} ---')
            self.current_target_index += 1 # 切换到下一个点
            self.publisher_.publish(Twist()) # 停一下
            return

        # 3. 计算角度差
        target_theta = math.atan2(target_y - current_y, target_x - current_x)
        angle_diff = target_theta - current_theta
        
        # 将角度差归一化到 -pi 到 pi
        while angle_diff > math.pi: angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi: angle_diff += 2.0 * math.pi

        cmd = Twist()
        
        # 4. 核心控制逻辑
        if abs(angle_diff) > 0.1:
            # 角度没对准：原地转（线速度为0，角速度跟角度差成正比）
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 * angle_diff 
        else:
            # 角度对准了：边走边微调（距离越近，速度越慢，防止冲过头）
            cmd.linear.x = 1.0 * distance 
            cmd.angular.z = 1.0 * angle_diff

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()