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
        self.sub_ball = self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.balls = {}
        self.current_pose = None
        self.targets = []
        self.current_target_index = 0
        self.moving = False
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("导航节点已启动，等待接收3个球的位置...")

    def ball_callback(self, msg):
        if msg.id not in self.balls:
            self.balls[msg.id] = (msg.x, msg.y)
            self.get_logger().info(f'已记录第 {msg.id} 个球: ({msg.x}, {msg.y})')
            
        if len(self.balls) == 3 and not self.moving and len(self.targets) == 0:
            self.targets = [self.balls[1], self.balls[2], self.balls[3]]
            self.moving = True
            self.get_logger().info("已收齐3个球，开始按 1->2->3 顺序导航！")

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if not self.moving or self.current_pose is None:
            return
            
        if self.current_target_index >= len(self.targets):
            return 
            
        target_x, target_y = self.targets[self.current_target_index]
        
        dist = math.sqrt((target_x - self.current_pose.x)**2 + (target_y - self.current_pose.y)**2)
        target_theta = math.atan2(target_y - self.current_pose.y, target_x - self.current_pose.x)
        angle_diff = target_theta - self.current_pose.theta
        
        while angle_diff > math.pi: angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi: angle_diff += 2.0 * math.pi
        
        cmd = Twist()
        
        if dist < 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_vel.publish(cmd)
            self.get_logger().info(f'到达球 {self.current_target_index+1}！实际坐标: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}), 误差: {dist:.2f}')
            self.current_target_index += 1
            if self.current_target_index >= len(self.targets):
                self.get_logger().info("所有球均已到达，任务完成！")
                self.moving = False
        else:
            if abs(angle_diff) > 0.2:
                cmd.linear.x = 0.0
                cmd.angular.z = 1.5 * angle_diff
            else:
                cmd.linear.x = 1.0 * dist
                cmd.angular.z = 1.5 * angle_diff
            self.pub_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()