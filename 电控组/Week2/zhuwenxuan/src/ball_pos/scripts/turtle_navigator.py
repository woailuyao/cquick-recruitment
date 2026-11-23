#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 2 · 闭环导航：订阅 /ball_pos，收齐 3 个球后按 1 → 2 → 3 依次走过去

思路（闭环）：
  1. 从 /turtle1/pose 拿到乌龟当前 (x, y, theta)
  2. 算出到目标的距离 dist 和方位差 diff
  3. |diff| 大  → 原地转（linear.x = 0），先对准
  4. |diff| 小  → 边走边微调，越近越慢（速度跟距离挂钩，防止冲过头）
  5. dist < 阈值 → 认为到了，打印误差，切下一个点
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from ball_pos.msg import BallPos

ORDER = [1, 2, 3]        # 必须按 1 → 2 → 3 顺序
ARRIVE_DIST = 0.30       # 到点阈值（考核要求 ≤0.5，取 0.3 留余量）
ANGLE_TOL = 0.08         # 方位差小于它就算「对准了」
MAX_LINEAR = 1.0         # 最大线速度 m/s
MIN_LINEAR = 0.15
MAX_ANGULAR = 2.0        # 最大角速度 rad/s
K_ANGLE = 2.5            # 角度比例系数
K_LINEAR = 1.0           # 距离比例系数


class TurtleNavigator(Node):

    def __init__(self):
        super().__init__('turtle_navigator')

        self.pose = None          # 乌龟当前位姿
        self.balls = {}           # id -> (x, y)
        self.results = []         # (id, 实际x, 实际y, 误差)
        self.index = 0            # 当前去第几个点
        self.goal = None
        self.goal_id = None
        self.finished = False

        # 订阅球坐标（任务 1 的发布端）
        self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)
        # 订阅乌龟位姿（闭环的关键：反馈）
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        # 发布速度指令
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 20 Hz 控制循环
        self.create_timer(0.05, self.control)

        self.get_logger().info('turtle_navigator 已启动，等待收齐 3 个球 ...')

    # ---------- 回调 ----------
    def on_ball(self, msg):
        if msg.id not in (1, 2, 3):
            return
        self.balls[int(msg.id)] = (float(msg.x), float(msg.y))
        if len(self.balls) == 3 and self.goal is None and not self.finished:
            self.get_logger().info('=== 已收齐 3 个球 ===')
            for i in ORDER:
                self.get_logger().info(
                    '   球 %d : (%.2f, %.2f)' % (i, self.balls[i][0], self.balls[i][1]))
            self.start_next()

    def on_pose(self, msg):
        self.pose = msg

    # ---------- 目标管理 ----------
    def start_next(self):
        if self.index >= len(ORDER):
            self.finish()
            return
        self.goal_id = ORDER[self.index]
        self.goal = self.balls[self.goal_id]
        self.get_logger().info(
            '--> 前往球 %d，目标坐标 (%.2f, %.2f)'
            % (self.goal_id, self.goal[0], self.goal[1]))

    # ---------- 控制循环 ----------
    def control(self):
        if self.finished or self.goal is None or self.pose is None:
            return

        dx = self.goal[0] - self.pose.x
        dy = self.goal[1] - self.pose.y
        dist = math.hypot(dx, dy)

        # 方位差归一化到 [-pi, pi]
        target_theta = math.atan2(dy, dx)
        diff = math.atan2(math.sin(target_theta - self.pose.theta),
                          math.cos(target_theta - self.pose.theta))

        tw = Twist()

        if dist < ARRIVE_DIST:
            # 到点：停车、记录、切下一个
            self.stop()
            self.get_logger().info(
                '到达球 %d   实际坐标 (%.3f, %.3f)   误差 %.3f'
                % (self.goal_id, self.pose.x, self.pose.y, dist))
            self.results.append((self.goal_id, self.pose.x, self.pose.y, dist))
            self.index += 1
            self.start_next()
            return

        if abs(diff) > ANGLE_TOL:
            # 方位差大：原地转，先把头对准
            tw.linear.x = 0.0
            tw.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, K_ANGLE * diff))
        else:
            # 对准了：边走边微调，越近越慢
            tw.linear.x = min(MAX_LINEAR, max(MIN_LINEAR, K_LINEAR * dist))
            tw.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, K_ANGLE * diff))

        self.cmd_pub.publish(tw)

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ---------- 收尾 ----------
    def finish(self):
        self.finished = True
        self.stop()
        self.get_logger().info('================ 任务 2 完成 ================')
        if not self.results:
            self.get_logger().warn('没有记录到任何到达点，检查发布端是否在跑')
            return
        total = 0.0
        for bid, x, y, d in self.results:
            total += d
            flag = 'OK' if d <= 0.5 else '超出阈值'
            self.get_logger().info(
                '球 %d   实际 (%.3f, %.3f)   误差 %.3f   %s' % (bid, x, y, d, flag))
        self.get_logger().info(
            '平均误差 %.3f    最大误差 %.3f'
            % (total / len(self.results), max(r[3] for r in self.results)))
        self.get_logger().info('三次到达全部完成，全程无人工按键')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
