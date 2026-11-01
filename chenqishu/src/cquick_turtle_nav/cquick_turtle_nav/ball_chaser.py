#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务2 · 导航节点：订阅 /ball_pos，收齐 3 个球后按 1 -> 2 -> 3 闭环走过去。

闭环三句话：
  1. 算出到目标点的距离 dist 和方位角差 angle_err；
  2. 角度差大 -> 原地转（linear.x = 0），先对准；
  3. 对准了 -> 边走边微调，越近越慢；dist 小于阈值就算到了，切下一个点。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose

from cquick_ball_msgs.msg import BallPos

# ===== 可调参数（调速度就改这里）=====
ORDER = [1, 2, 3]        # 必须按 1 -> 2 -> 3，不许跳、不许倒
ARRIVE_DIST = 0.12       # 到点阈值（米），< 这个值算到达；判定要求误差 <= 0.5
ALIGN_ANGLE = 0.35       # 角度差大于这个值先原地转（rad，约 20°）
MAX_LINEAR = 0.7         # 最大线速度 m/s
MAX_ANGULAR = 2.0        # 最大角速度 rad/s
K_LINEAR = 0.9           # 线速度 = K_LINEAR * 距离  -> 越近越慢
K_ANGULAR = 2.5          # 角速度 = K_ANGULAR * 角度差
TIMEOUT_PER = 70.0       # 单个点超时保护（秒），超了就报错并跳下一个


def clamp(value, limit):
    return max(-limit, min(limit, value))


def normalize_angle(a):
    """把角度折到 [-pi, pi]，否则 -3.1 和 +3.1 会被当成差很多。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class BallChaser(Node):

    def __init__(self):
        super().__init__('ball_chaser')

        self.balls = {}          # id -> (x, y)，收一个存一个
        self.targets = []        # 收齐后按 ORDER 排好的 [(id, x, y), ...]
        self.index = -1          # 当前在追第几个点
        self.pose = None         # 乌龟当前位姿
        self.started = False
        self.finished = False
        self.target_start = 0.0
        self.report = []         # 每个点的实际到达记录，最后汇总打印

        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)

        # 20 Hz 控制循环
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('导航节点已启动，等待 /ball_pos 收齐 3 个球 ...')

    # ---------- 回调 ----------
    def on_pose(self, msg):
        self.pose = msg

    def on_ball(self, msg):
        if msg.id not in self.balls:
            self.get_logger().info(
                f'收到球 {msg.id}: ({msg.x:.1f}, {msg.y:.1f})，已收 {len(self.balls) + 1}/3')
        self.balls[msg.id] = (msg.x, msg.y)

        # 收齐 3 个再动（发布端 2 Hz，最多等 1 秒）
        if self.started or len(self.balls) < 3:
            return
        if any(i not in self.balls for i in ORDER):
            return

        self.targets = [(i, self.balls[i][0], self.balls[i][1]) for i in ORDER]
        self.started = True
        self.get_logger().info('3 个球已收齐，开始按 1 -> 2 -> 3 依次前往')
        self.next_target()

    # ---------- 目标切换 ----------
    def next_target(self):
        self.index += 1
        if self.index >= len(self.targets):
            self.finish()
            return
        ball_id, x, y = self.targets[self.index]
        self.target_start = self.now_sec()
        self.get_logger().info(
            f'==> 前往球 {ball_id}：目标 ({x:.2f}, {y:.2f})')

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def stop_turtle(self):
        self.vel_pub.publish(Twist())   # 全 0 = 停下

    # ---------- 核心：闭环控制 ----------
    def control_loop(self):
        if (not self.started) or self.finished or self.pose is None:
            return

        ball_id, tx, ty = self.targets[self.index]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)

        # 到了
        if dist < ARRIVE_DIST:
            self.arrive(ball_id, tx, ty, dist)
            return

        # 超时保护（正常不会触发，防卡死）
        if self.now_sec() - self.target_start > TIMEOUT_PER:
            self.get_logger().warn(
                f'球 {ball_id} 超时未到达，当前误差 {dist:.2f}，跳到下一个点')
            self.stop_turtle()
            self.next_target()
            return

        angle_err = normalize_angle(math.atan2(dy, dx) - self.pose.theta)

        cmd = Twist()
        if abs(angle_err) > ALIGN_ANGLE:
            # 先对准再走：原地转，线速度为 0
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(K_ANGULAR * angle_err, MAX_ANGULAR)
        else:
            # 边走边转：越近越慢，避免冲过头
            cmd.linear.x = min(K_LINEAR * dist, MAX_LINEAR)
            cmd.angular.z = clamp(K_ANGULAR * angle_err, 1.2)
        self.vel_pub.publish(cmd)

    # ---------- 到点记录 ----------
    def arrive(self, ball_id, tx, ty, dist):
        self.stop_turtle()
        ax, ay = self.pose.x, self.pose.y
        err = math.hypot(tx - ax, ty - ay)   # 用实际位姿复算误差
        self.report.append((ball_id, tx, ty, ax, ay, err))
        flag = 'OK' if err <= 0.5 else '超限'
        self.get_logger().info(
            f'*** 到达球 {ball_id}，实际坐标 ({ax:.3f}, {ay:.3f})，'
            f'误差 {err:.3f} [{flag}] ***')
        self.next_target()

    def finish(self):
        self.finished = True
        self.stop_turtle()
        self.destroy_timer(self.control_timer)
        self.get_logger().info('========== 三个球全部到达，误差汇总 ==========')
        for ball_id, tx, ty, ax, ay, err in self.report:
            self.get_logger().info(
                f'球 {ball_id}: 目标 ({tx:.2f}, {ty:.2f}) 实际 ({ax:.3f}, {ay:.3f}) '
                f'误差 {err:.3f}' + ('  <= 0.5 达标' if err <= 0.5 else '  > 0.5 不达标'))
        self.get_logger().info('任务完成。节点保持在运行状态，按 Ctrl+C 退出。')


def main(args=None):
    rclpy.init(args=args)
    node = BallChaser()
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
