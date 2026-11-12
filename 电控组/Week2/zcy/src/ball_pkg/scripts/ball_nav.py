#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pkg.msg import BallPos

BALLS = {1: (2.0, 3.0), 2: (5.5, 8.0), 3: (9.0, 4.0)}

class BallNavigator(Node):
    def __init__(self):
        super().__init__('ball_navigator')
        self.pose = None
        self.ball_positions = {}
        self.order = [1, 2, 3]
        self.current_idx = 0
        self.arrived = False

        # 发布速度控制小乌龟，订阅乌龟位置，订阅球的位置
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.ball_sub = self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)

        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz 控制频率
        self.get_logger().info('导航节点已启动，等待收齐 3 个球...')

    def ball_cb(self, msg):
        self.ball_positions[msg.id] = (msg.x, msg.y)
        if len(self.ball_positions) == 3 and not self.arrived:
            self.get_logger().info('已收齐 3 个球，开始导航')

    def pose_cb(self, msg):
        self.pose = msg

    def control_loop(self):
        if len(self.ball_positions) < 3 or self.pose is None or self.arrived:
            return

        target_id = self.order[self.current_idx]
        tx, ty = BALLS[target_id]
        x, y, theta = self.pose.x, self.pose.y, self.pose.theta

        distance = math.sqrt((tx - x)**2 + (ty - y)**2)
        angle_diff = math.atan2(ty - y, tx - x) - theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        cmd = Twist()

        if distance < 0.15:   # 到达目标点
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'到达球 {target_id}！实际坐标 ({x:.2f}, {y:.2f})，误差 {distance:.2f}')
            self.current_idx += 1
            if self.current_idx >= len(self.order):
                self.arrived = True
                self.get_logger().info('三个球全部到达，导航结束')
            return

        if abs(angle_diff) > 0.3:          # 角度差大，原地转
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 * angle_diff
        else:                               # 角度对上了，边走边微调
            cmd.linear.x = 1.0 * distance
            cmd.angular.z = 2.0 * angle_diff

        cmd.linear.x = min(cmd.linear.x, 1.5)
        cmd.angular.z = max(min(cmd.angular.z, 2.0), -2.0)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BallNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()