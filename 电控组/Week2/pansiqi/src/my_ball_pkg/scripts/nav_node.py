#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务2：导航节点

作用：
    1. 订阅任务1发布者发来的 /ball_pos，收齐 3 个球的坐标；
    2. 订阅 turtlesim 的 /turtle1/pose，拿到乌龟当前 (x, y, theta)；
    3. 按 id 1 -> 2 -> 3 的顺序，用闭环控制发布 Twist 到 /turtle1/cmd_vel，
       让乌龟依次走到三个目标点；
    4. 每到一个点，打印实际坐标和误差。

闭环思路：
    不是“按时间估算走多久”，而是每一刻都用当前位姿重新算：
        距离 = 目标点 - 当前点
        方位角差 = 目标方向 - 当前朝向
    方位差大 -> 先原地转对准；
    方位差小 -> 边走边微调，越接近目标线速度越小，避免冲过头。
"""

import math

import rclpy
from rclpy.node import Node

from my_ball_pkg.msg import BallPos          # 任务1自定义的球位置消息
from turtlesim.msg import Pose               # 乌龟位姿：x, y, theta
from geometry_msgs.msg import Twist          # 速度指令：linear.x, angular.z


class NavNode(Node):
    """导航节点：收球坐标 + 读乌龟位姿 -> 算速度 -> 发 Twist。"""

    def __init__(self):
        super().__init__('nav_node')

        # ---------- 内部状态 ----------
        # self.balls 用字典按 id 存球坐标：{1: (2.0, 3.0), 2: (5.5, 8.0), ...}
        # 用字典而不是列表，是因为话题上每条消息只带一个球，
        # 靠 id 区分“这是第几个球”，收齐 3 个再开始。
        self.balls = {}

        # 乌龟当前位姿，类型是 turtlesim/msg/Pose，包含 x, y, theta。
        # 初始为 None，表示还没收到 /turtle1/pose，不能开始控制。
        self.pose = None

        # 当前正在追的目标点 (x, y)。
        # None 表示“还没选目标”或“刚到一个点，需要选下一个”。
        self.current_target = None

        # 强制顺序：必须按 1 -> 2 -> 3 走，不依赖消息到达的先后。
        self.order = [1, 2, 3]

        # 当前走到 order 的第几个（0 表示还没开始，1 表示已到球1，以此类推）。
        self.index = 0

        # ---------- 三进一出 ----------
        # 订阅1：任务1的球坐标话题 /ball_pos，收到后进 ball_cb
        self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)

        # 订阅2：乌龟位姿话题 /turtle1/pose，收到后进 pose_cb
        # 这是闭环的“反馈来源”，没有它就只能开环估算。
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        # 发布：速度指令 /turtle1/cmd_vel，替代第一周的键盘遥控节点。
        # turtlesim_node 订阅这个话题，收到 Twist 就动。
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 控制循环：每 0.05 秒（20 Hz）跑一次 control_loop。
        # 频率越高，控制越平滑，但也不用太高，20 Hz 足够。
        self.create_timer(0.05, self.control_loop)

    # ---------- 回调：收数据 ----------

    def ball_cb(self, msg):
        """每收到一条 BallPos，就按 id 存进字典。"""
        # msg.id 是 int32，msg.x / msg.y 是 float32。
        # 同一个 id 重复收到会覆盖，没关系，坐标是固定的。
        self.balls[msg.id] = (msg.x, msg.y)

    def pose_cb(self, msg):
        """每收到一条乌龟位姿，就更新 self.pose。"""
        # msg.x, msg.y 是乌龟当前位置（乌龟坐标单位，0~11）。
        # msg.theta 是乌龟当前朝向（弧度，0 指向右，逆时针为正）。
        self.pose = msg

    # ---------- 控制循环 ----------

    def control_loop(self):
        """20 Hz 主循环：判断能不能开始、算距离和角度、发 Twist。"""

        # 条件1：必须收齐 3 个球（len == 3，用 < 3 判断更稳）。
        # 条件2：必须至少收到一次乌龟位姿，否则不知道自己在哪。
        # 任一不满足就先不动，等下一轮。
        if len(self.balls) < 3 or self.pose is None:
            return

        # 如果当前没有目标，就按 order 选下一个。
        # 例如刚开始 index=0，会选球1；到球1后 index=1，下一轮会选球2。
        if self.current_target is None:
            self.pick_next()

        # 如果 pick_next 之后还是没有目标，说明 3 个都走完了，
        # 或者 order 里的 id 还没收到（正常收齐 3 个不会发生）。
        # 这时发一个全零 Twist 让乌龟停下，然后直接返回。
        if self.current_target is None:
            self.stop()
            return

        # ---------- 算距离和方位 ----------
        tx, ty = self.current_target          # 目标点
        dx = tx - self.pose.x                 # x 方向差值
        dy = ty - self.pose.y                 # y 方向差值
        distance = math.sqrt(dx * dx + dy * dy)  # 到目标点的直线距离

        # ---------- 到点判定 ----------
        # 距离小于阈值就认为“到了”。
        # 阈值 0.1 比考核要求的 0.5 更严，留余量，防止实际误差超限。
        if distance < 0.1:
            ball_id = self.order[self.index]
            self.get_logger().info(
                f'到达球 {ball_id}，实际 ({self.pose.x:.2f}, {self.pose.y:.2f})，误差 {distance:.2f}')

            # 切下一个目标：
            # index 加 1，current_target 清空，下一轮 control_loop 会重新 pick_next。
            self.index += 1
            self.current_target = None

            # 如果 index 已经超过 order 长度，说明 3 个都到了。
            if self.index >= len(self.order):
                self.get_logger().info('三个球全部到达')

            # 本轮不再发速度，等下一轮选新目标。
            return

        # ---------- 算方位角误差 ----------
        # target_theta：从当前位置指向目标点的方向角（弧度）。
        # atan2(dy, dx) 返回 [-pi, pi]，0 指向右，逆时针为正。
        target_theta = math.atan2(dy, dx)

        # angle_error：目标方向 - 当前朝向。
        # 但直接相减可能超出 [-pi, pi]，例如当前 170°、目标 -170°，
        # 直接减是 340°，实际应该转 -20°。
        # 用 atan2(sin, cos) 归一化到 [-pi, pi]，保证走最短转向。
        angle_error = target_theta - self.pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # 新建一条速度指令，默认 linear.x=0, angular.z=0。
        cmd = Twist()

        # ---------- 先对准，再走 ----------
        # 方位差大于 0.1 弧度（约 5.7°）时，认为“还没对准”。
        # 这时只原地转：linear.x = 0，angular.z 与角度误差成正比。
        # 为什么要先对准？因为乌龟是差速底盘，朝向不对时直接给线速度，
        # 会走弧线甚至绕圈，不如先转正再直走稳。
        if abs(angle_error) > 0.1:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0 * angle_error
        else:
            # 已经对准：边走边微调。
            # linear.x 与距离成正比，越近越小，避免冲过头；
            # min(..., 0.8) 是限速，最大 0.8，防止一开始冲太快。
            cmd.linear.x = min(0.5 * distance, 0.8)

            # 走的过程中仍然用角速度微调方向，系数 1.5 比原地转时大，
            # 让它在走直线时也能快速修正小偏差。
            cmd.angular.z = 1.5 * angle_error

        # 把这一轮算出的速度指令发到 /turtle1/cmd_vel。
        # turtlesim_node 订阅这个话题，收到后更新乌龟位置。
        self.pub.publish(cmd)

    # ---------- 辅助方法 ----------

    def pick_next(self):
        """按 order 选下一个目标点。"""
        # 如果 index 已经超出 order 长度，说明全部走完，不选。
        if self.index >= len(self.order):
            return

        # 取出当前该去的球 id，例如 index=0 时 ball_id=1。
        ball_id = self.order[self.index]

        # 如果这个 id 的坐标已经收到，就设为当前目标。
        # 正常收齐 3 个后一定在；这里加判断是防御性写法。
        if ball_id in self.balls:
            self.current_target = self.balls[ball_id]

    def stop(self):
        """发一条全零 Twist，让乌龟停下。"""
        # Twist() 默认 linear.x=0, angular.z=0，即“不走也不转”。
        self.pub.publish(Twist())


def main(args=None):
    # 初始化 rclpy
    rclpy.init(args=args)

    # 创建节点实例
    node = NavNode()

    # spin：让节点一直活着，等话题消息、等定时器触发。
    # 没有 spin，节点创建完就退出，回调永远不会执行。
    rclpy.spin(node)

    # 退出时清理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()