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
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.sub_ball = self.create_subscription(BallPos, '/ball_pos', self.ball_cb, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.targets = {}
        self.target_list = []
        self.current_target_idx = 0
        self.is_moving = False
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("导航节点已启动，等待接收3个球的位置...")

    def ball_cb(self, msg):
        if msg.id not in self.targets:
            self.targets[msg.id] = (msg.x, msg.y)
            self.get_logger().info(f"已记录球 {msg.id}: ({msg.x}, {msg.y})")
            
        if len(self.targets) == 3 and not self.is_moving:
            self.target_list = [self.targets[1], self.targets[2], self.targets[3]]
            self.get_logger().info("收齐3个球，开始导航！")
            self.is_moving = True

    def pose_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def control_loop(self):
        if not self.is_moving or self.current_target_idx >= 3:
            return

        target_x, target_y = self.target_list[self.current_target_idx]
        
        dist = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        
        if dist < 0.15:
            self.get_logger().info(f"到达球 {self.current_target_idx + 1}，实际坐标 ({self.current_x:.2f}, {self.current_y:.2f})，误差 {dist:.2f}")
            self.current_target_idx += 1
            if self.current_target_idx >= 3:
                self.get_logger().info("所有目标点已到达！任务完成！")
                self.is_moving = False
                self.publisher_.publish(Twist())
            return

        angle_to_target = math.atan2(target_y - self.current_y, target_x - self.current_x)
        angle_diff = angle_to_target - self.current_theta
        while angle_diff > math.pi: angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi: angle_diff += 2.0 * math.pi

        cmd = Twist()
        if abs(angle_diff) > 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 * angle_diff
        else:
            cmd.linear.x = min(1.0 * dist, 1.5)
            cmd.angular.z = 2.0 * angle_diff
            
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
