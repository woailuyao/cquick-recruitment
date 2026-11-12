#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务2 · 导航节点：订阅 /ball_pos 收齐 3 个球，按 1 → 2 → 3 依次让小乌龟走过去。

路线 A（闭环）：订阅 /turtle1/pose 拿当前位姿，实时算「距离 + 方位差」：
  1. 方位差大  -> 原地转（linear.x = 0，angular.z 按偏差给）
  2. 方位对上了 -> 边走边微调，越接近目标速度越小，避免冲过头
  3. 距离 < 阈值 -> 认为到达，打印实际坐标和误差，切下一个点
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.msg import Pose

from ball_comm.msg import BallPos

ORDER = [1, 2, 3]          # 必须按 1 -> 2 -> 3
MAX_V = 0.8                # 最大线速度 m/s
MIN_V = 0.12               # 接近目标时的最小线速度，太小会停住
MAX_W = 1.6                # 最大角速度 rad/s
GOAL_TOL = 0.10            # 到点阈值（细则要求最终误差 <= 0.5，这里留足余量）
ANGLE_TOL = 0.12           # 方位差小于它才开始走（先对准再走）


class BallNavigator(Node):
    def __init__(self):
        super().__init__('ball_navigator')
        self.balls = {}                 # id -> (x, y)
        self.pose = None                # 当前位姿
        self.phase = 'wait_pose'        # wait_pose -> wait_balls -> run -> done
        self.idx = 0                    # 当前去第几个球
        self.results = []               # [(id, 目标, 实际, 误差)]

        self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.clear_cli = self.create_client(Empty, '/clear')

        self.create_timer(0.05, self.control)     # 20 Hz 控制循环
        self.get_logger().info('导航节点已启动：等待 /turtle1/pose 与 /ball_pos ...')

    # ---------- 回调 ----------
    def on_ball(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        # 只在前 3 条（收球阶段）打日志，之后不再刷屏
        if self.phase == 'wait_balls' and len(self.balls) <= 3:
            self.get_logger().info('收到球 id=%d (%.1f, %.1f)  ->  %d/3'
                                   % (msg.id, msg.x, msg.y, len(self.balls)))

    def on_pose(self, msg):
        self.pose = msg
        if self.phase == 'wait_pose':
            self.phase = 'wait_balls'
            self.get_logger().info('拿到乌龟位姿 (%.2f, %.2f, %.2f)' % (msg.x, msg.y, msg.theta))

    # ---------- 控制 ----------
    def control(self):
        if self.phase == 'done':
            return
        if self.phase == 'wait_pose' or self.pose is None:
            return
        if self.phase == 'wait_balls':
            if len(self.balls) >= 3:
                self.start_run()
            return
        self.drive_to_target()

    def start_run(self):
        self.get_logger().info('3 个球已收齐：%s' % (sorted(self.balls.items()),))
        # 开始前清屏（这也是一次服务调用：请求 -> 响应，只调一次）
        if self.clear_cli.wait_for_service(timeout_sec=2.0):
            req = Empty.Request()
            self.clear_cli.call_async(req)
            self.get_logger().info('已调用服务 /clear 清空轨迹')
        else:
            self.get_logger().warn('/clear 服务不可用，跳过清屏')
        self.phase = 'run'
        self.idx = 0

    def drive_to_target(self):
        ball_id = ORDER[self.idx]
        tx, ty = self.balls[ball_id]
        dx, dy = tx - self.pose.x, ty - self.pose.y
        dist = math.hypot(dx, dy)
        cmd = Twist()

        if dist <= GOAL_TOL:
            # 到达：停住、记录、切下一个
            self.cmd_pub.publish(cmd)
            self.results.append((ball_id, (tx, ty), (self.pose.x, self.pose.y), dist))
            self.get_logger().info('到达球 %d，实际坐标 (%.2f, %.2f)，误差 %.2f'
                                   % (ball_id, self.pose.x, self.pose.y, dist))
            self.idx += 1
            if self.idx >= len(ORDER):
                self.finish()
            return

        # 目标方位 - 当前朝向，归一化到 [-pi, pi]
        target_theta = math.atan2(dy, dx)
        err = math.atan2(math.sin(target_theta - self.pose.theta),
                         math.cos(target_theta - self.pose.theta))

        if abs(err) > ANGLE_TOL:
            # 方位差大：先原地转，不往前走
            cmd.linear.x = 0.0
            cmd.angular.z = max(-MAX_W, min(MAX_W, 2.5 * err))
        else:
            # 方位对上：越近越慢，同时轻微修正方向
            cmd.linear.x = max(MIN_V, min(MAX_V, 1.2 * dist))
            cmd.angular.z = max(-MAX_W, min(MAX_W, 3.0 * err))
        self.cmd_pub.publish(cmd)

    def finish(self):
        self.phase = 'done'
        self.cmd_pub.publish(Twist())          # 停车
        self.get_logger().info('=== 三个球全部到达，误差汇总 ===')
        for bid, tgt, act, err in self.results:
            self.get_logger().info('球 %d：目标 (%.1f, %.1f) 实际 (%.2f, %.2f) 误差 %.2f'
                                   % (bid, tgt[0], tgt[1], act[0], act[1], err))
        worst = max(r[3] for r in self.results)
        ok = '全部达标（每个点误差 <= 0.5）' if worst <= 0.5 else '有误差超限，需要调参'
        self.get_logger().info('最大误差 %.2f -> %s' % (worst, ok))


def main():
    rclpy.init()
    node = BallNavigator()
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
