#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from cquick_week2.msg import BallPos

# 细则是规定好的，不能改
TARGETS = {1: (2.0, 3.0), 2: (5.5, 8.0), 3: (9.0, 4.0)}
TOL = 0.5          # 到达判定阈值（米）
ORDER = [1, 2, 3]  # 必须按这个顺序走

class GoToBalls(Node):
    def __init__(self):
        super().__init__('go_to_balls')
        # 发速度指令给乌龟
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 收球的位置
        self.create_subscription(BallPos, '/ball_pos', self.on_ball, 10)
        # 收乌龟自己的位姿（闭环的关键）
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)

        self.balls = {}       # 字典：id -> (x, y)
        self.pose = None      # 乌龟当前位姿
        self.step = 0         # 当前要去的第几个球
        self.finished = False
        self.create_timer(0.05, self.control)   # 20 Hz 控制循环
        self.get_logger().info('等待收齐 3 个球的位置...')

    def on_ball(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        if len(self.balls) == 3 and self.step == 0:
            self.get_logger().info(
                f'已收齐 3 个球：{self.balls}，开始按 1→2→3 前进')

    def on_pose(self, msg):
        self.pose = msg

    def control(self):
        if self.finished or self.pose is None or len(self.balls) < 3:
            return                       # 条件不满足就什么都不做

        n = ORDER[self.step]
        tx, ty = TARGETS[n]
        dx, dy = tx - self.pose.x, ty - self.pose.y     # 目标方向向量
        dist = math.hypot(dx, dy)

        if dist < TOL:                   # 到达
            self.publish_stop()
            self.get_logger().info(
                f'到达第 {n} 个球，实际坐标 ({self.pose.x:.2f}, {self.pose.y:.2f})，'
                f'误差 {dist:.2f}')
            self.step += 1
            if self.step >= len(ORDER):
                self.finished = True
                self.get_logger().info('三个球全部到达，任务完成')
            return
        target = math.atan2(dy, dx)
        err = math.atan2(math.sin(target - self.pose.theta),
                         math.cos(target - self.pose.theta))

        t = Twist()
        if abs(err) > 0.15:
            # 方向差得多，原地转（只用角速度）
            t.linear.x = 0.0
            t.angular.z = max(-2.0, min(2.0, 2.0 * err))
        else:
            # 方向对准了，前进 + 微调；越近越慢
            t.linear.x = min(1.5, 2.0 * dist)
            t.angular.z = max(-1.0, min(1.0, 2.0 * err))
        self.cmd_pub.publish(t)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())    # 全零速度 = 停

def main():
    rclpy.init()
    node = GoToBalls()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()