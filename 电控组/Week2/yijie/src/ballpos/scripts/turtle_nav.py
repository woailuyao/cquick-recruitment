#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ballpos.msg import BallPos
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtleNav(Node):
    def __init__(self):
        super().__init__('turtle_nav')
        
        # 1. 订阅球的位置 (任务1的发布者)
        self.sub_ball = self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)
        
        # 2. 订阅乌龟当前的位置 (闭环必需)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # 3. 发布速度指令给乌龟
        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 初始化变量
        self.balls = {}         # 用字典存球的位置 {id: (x, y)}
        self.current_pose = None # 乌龟当前的位置
        self.current_goal_id = 1 # 当前要去第几个球
        self.is_ready = False    # 是否收齐了3个球

    def ball_callback(self, msg):
        # 收到球的位置，存入字典，按 id 区分
        self.balls[msg.id] = (msg.x, msg.y)
        if len(self.balls) == 3 and not self.is_ready:
            self.get_logger().info('收齐3个球！准备出发！')
            self.is_ready = True

    def pose_callback(self, msg):
        self.current_pose = msg
        if not self.is_ready or self.current_pose is None:
            return

        # 如果 1、2、3 都走完了，就停下
        if self.current_goal_id > 3:
            self.stop_turtle()
            return

        # 获取目标点坐标
        target_x, target_y = self.balls[self.current_goal_id]

        # --- 核心闭环控制逻辑 ---
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2) # 算距离

        # 如果距离小于 0.1，说明到了
        if distance < 0.1:
            self.get_logger().info(f'到达球 {self.current_goal_id}，实际坐标 ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})，误差 {distance:.2f}')
            self.current_goal_id += 1 # 切换到下一个目标
            self.stop_turtle()
            return

        # 算目标角度和当前乌龟朝向的差值
        goal_theta = math.atan2(dy, dx)
        angle_diff = goal_theta - self.current_pose.theta

        # 将角度差值归一化到 [-pi, pi] (防止乌龟绕远路)
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        cmd = Twist()
        # 策略：先对准再走
        if abs(angle_diff) > 0.3: # 角度差大，原地转
            cmd.linear.x = 0.0
            cmd.angular.z = 2.0 * angle_diff
        else: # 角度对准了，边走边微调，越近越慢
            cmd.linear.x = min(1.0 * distance, 1.5) # 限制最大速度
            cmd.angular.z = 2.0 * angle_diff

        self.pub_cmd.publish(cmd)

    def stop_turtle(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
