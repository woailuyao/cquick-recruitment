#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_task_pkg.msg import BallPos
import math

class BallChaser(Node):
    def __init__(self):
        super().__init__('ball_chaser')
        
        # 1. 创建发布者：控制乌龟速度
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 2. 创建订阅者：获取乌龟当前位置
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # 3. 创建订阅者：接收球的位置
        self.ball_sub = self.create_subscription(BallPos, '/ball_pos', self.ball_callback, 10)
        
        # 状态变量
        self.current_pose = None
        self.balls = {} # 用字典存球，按 id 去重： {1: (x,y), 2: (x,y), 3: (x,y)}
        self.is_moving = False # 是否开始移动
        self.target_idx = 1    # 当前要去的球号
        
        # 定时器：以 10Hz 频率控制
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("等待接收 3 个球的位置...")

    def pose_callback(self, msg):
        self.current_pose = msg

    def ball_callback(self, msg):
        # 保存球坐标（收到重复的会覆盖，没关系）
        self.balls[msg.id] = (msg.x, msg.y)
        
        # 如果收齐了 3 个球，且还没开始移动，则开始移动
        if len(self.balls) >= 3 and not self.is_moving:
            self.get_logger().info("已收齐 3 个球，准备出发！")
            self.is_moving = True

    def control_loop(self):
        if not self.is_moving or self.current_pose is None:
            return # 没准备好，不动
            
        # 如果 3 个球都去过了，停止并退出
        if self.target_idx > 3:
            self.stop_turtle()
            return

        # 获取目标球坐标
        target_x, target_y = self.balls[self.target_idx]
        
        # 计算距离和角度差
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # 目标朝向 (atan2 返回 -pi 到 pi)
        target_theta = math.atan2(dy, dx)
        # 计算角度误差，并归一化到 -pi 到 pi
        angle_diff = target_theta - self.current_pose.theta
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        cmd = Twist()

        # 判定：到达目标点（误差 < 0.5）
        if distance < 0.5:
            self.stop_turtle()
            self.get_logger().info(f'到达第 {self.target_idx} 个球，实际坐标 ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})，误差 {distance:.3f}')
            self.target_idx += 1 # 切换到下一个目标
            return

        # 核心控制逻辑：先对准，再走
        if abs(angle_diff) > 0.1: # 角度偏差大，原地转
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 if angle_diff > 0 else -1.5 # 根据偏差方向转
        else: # 角度对上了，边走边微调
            cmd.linear.x = 1.0 * distance # 越近速度越慢 (比例控制)
            cmd.angular.z = 2.0 * angle_diff # 微调角度
            
        self.cmd_pub.publish(cmd)

    def stop_turtle(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BallChaser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
