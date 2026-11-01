#!/usr/bin/env python3
"""订阅 `/turtle1/pose` 拿到乌龟当前 (x, y, θ)**，再根据「目标点 − 当前点」实时算该往哪走"""
import math
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# 标准速度信息
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pos_pkg.msg import BallPos

class BallNav(Node):
    def __init__(self):
        # 注册节点
        super().__init__('ball_nav')
        # 订阅与发布
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(BallPos, '/ball_pos', self.on_ball, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        # 定时器
        self.rate = 20
        self.create_timer(1 / self.rate, self.control)

        self.balls = {}
        self.targets = []
        self.idx = 0
        self.pose = None

        self.con_time = time.monotonic()

        # 控制速度
        self.linear_x = 0.0
        self.angular_z = 0.0

        # 可调参数
        self.angular_z_k = 1.2          # 角速度指数增益
        self.linear_x_k = 2.0           # 线速度指数增益
        self.err_deadband = 0.03        # 角误差死区

        # 机器人物理限制
        self.max_linear_x = 1.0
        self.max_linear_x_a = 1.0
        self.max_angular_z = math.pi/4

    def on_ball(self, msg):
        # 用消息里的 id 当键（不能写内置函数 id）
        self.balls[msg.id] = (msg.x, msg.y)
        # 收齐 3 个后按 1→2→3 生成目标列表，只生成一次
        if len(self.balls) == 3 and len(self.targets) != 3:
            self.targets = [self.balls[i] for i in sorted(self.balls)]
            self.get_logger().info(f"已收齐 3 个球：{self.targets}")

    def on_pose(self, msg):
        self.pose = (msg.x, msg.y, msg.theta)

    def control(self):
        if self.pose == None or len(self.targets) != 3 or self.idx == 3:
            if self.idx == 3 and self.linear_x > 0.0:
                # 三个球都走完：发一次零速度刹停，避免乌龟一直往前冲
                self.linear_x = 0.0
                self.angular_z = 0.0
                self.cmd_pub.publish(Twist())
        else:
            now = time.monotonic()
            dt = now - self.con_time                            # 秒（仅用于日志观察）

            d_delay = self.linear_x * (1.0 / self.rate)
            d_break = pow(self.max_linear_x,2) / (2 * self.max_linear_x_a)
            d_total = d_break + d_delay

            dx = self.targets[self.idx][0] - self.pose[0]
            dy = self.targets[self.idx][1] - self.pose[1]
            d  = math.hypot(dx, dy)                             # 到目标点的距离
            angle = math.atan2(dy, dx)                          # 目标点相对乌龟的方位
            err = math.atan2(math.sin(angle - self.pose[2]),    # 角误差，归一化到 [-π, π]
                            math.cos(angle - self.pose[2]))

            # 线速度
            if d > d_total:
                v = self.max_linear_x
            else:
                v = math.sqrt(2 * self.max_linear_x_a * d)
                d_rem = d - v * (1.0 / self.rate)
                if d_rem > 0:
                    v = math.sqrt(2 * self.max_linear_x_a * d_rem)
                else:
                    v = self.max_linear_x * (1 - math.exp(-self.linear_x_k * d_rem))

            v *= max(0.0, math.cos(err))

            if abs(err) > 1.2:
                v = 0.0

            # 角速度
            w = self.max_angular_z * (1 - math.exp(-self.angular_z_k * err))
            if abs(err) < self.err_deadband:
                w = 0.0
            w = max(-self.max_angular_z, min(self.max_angular_z, w))

            self.linear_x = max(0.0, v)
            self.angular_z = w


            msg = Twist()
            msg.linear.x = self.linear_x
            msg.angular.z = self.angular_z
            self.cmd_pub.publish(msg)

            if d <= 0.3:
                self.idx += 1
                self.get_logger().info(f'到达第 {self.idx} 个球，实际坐标 ({self.pose[0]:.3f}, {self.pose[1]:.3f})，误差 {d:.3f}')
                self.cmd_pub.publish(Twist())   # 到点先刹停，避免带着速度切下一个目标

            self.con_time = time.monotonic()

def main(args=None):
    rclpy.init(args=args)
    node = BallNav()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
