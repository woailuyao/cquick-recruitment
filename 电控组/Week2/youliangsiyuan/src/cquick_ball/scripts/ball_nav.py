#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务2 导航：订阅 ball_pos + /turtle1/pose，用 Twist 按 1->2->3 走过去。"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from cquick_ball.msg import BallPos

# ===== 可调参数 =====
MAX_LINEAR = 0.5
MAX_ANGULAR = 1.5
ANGLE_TOL = 0.5
SLOW_DIST = 0.8
ARRIVE_DIST = 0.15
CTRL_HZ = 20.0
TARGETS = [1, 2, 3]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class NavToBalls(Node):
    def __init__(self):
        super().__init__('nav_to_balls')
        self.balls = {}
        self.pose = None
        self.results = {}
        self.index = 0
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.timer = self.create_timer(1.0 / CTRL_HZ, self.control_cb)

        self.get_logger().info('nav_to_balls 启动，等待 3 个球坐标...')

    def ball_cb(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        self.get_logger().info(f'收到球 {msg.id} ({msg.x:.2f}, {msg.y:.2f})')

    def pose_cb(self, msg):
        self.pose = (msg.x, msg.y, msg.theta)

    def control_cb(self):
        if self.done:
            return
        # 收齐 3 个球 + 拿到过位姿才动
        if len(self.balls) < 3 or self.pose is None:
            return
        if self.index >= len(TARGETS):
            self.stop()
            self.done = True
            self.print_summary()
            return

        goal_id = TARGETS[self.index]
        goal = self.balls.get(goal_id)
        if goal is None:
            return

        x, y, theta = self.pose
        dx = goal[0] - x
        dy = goal[1] - y
        dist = math.hypot(dx, dy)                              # 距离 = 勾股
        heading_err = wrap_angle(math.atan2(dy, dx) - theta)    # 方位差

        cmd = Twist()
        if dist <= ARRIVE_DIST:
            # 到点：停车、记录误差、切下一个
            self.stop()
            self.results[goal_id] = (x, y, dist)
            self.get_logger().info(
                f'到达球 {goal_id}，实际坐标 ({x:.2f}, {y:.2f})，误差 {dist:.2f}')
            self.index += 1
            return

        if abs(heading_err) > ANGLE_TOL:
            # 方位差大 → 原地转，先对准
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(2.0 * heading_err, -MAX_ANGULAR, MAX_ANGULAR)
        else:
            # 对准了 → 前进，越近越慢
            speed = clamp(0.6 * dist, 0.05, MAX_LINEAR)
            if dist < SLOW_DIST:
                speed *= max(0.3, dist / SLOW_DIST)
            cmd.linear.x = speed
            cmd.angular.z = clamp(1.5 * heading_err, -MAX_ANGULAR, MAX_ANGULAR)

        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def print_summary(self):
        self.get_logger().info('===== 三点误差汇总 =====')
        for goal_id in TARGETS:
            if goal_id in self.results:
                x, y, err = self.results[goal_id]
                self.get_logger().info(
                    f'球 {goal_id}: 实际 ({x:.2f}, {y:.2f})，误差 {err:.2f}')
        self.get_logger().info('===== 全部完成 =====')


def main():
    rclpy.init()
    node = NavToBalls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()