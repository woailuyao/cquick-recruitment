#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务二：订阅 /ball_pos，收齐 3 个球后让乌龟按 1 -> 2 -> 3 依次走过去。

相对原版修掉的 4 个问题：
1. 到达第 3 个点后只打印一次汇总，然后自动关定时器 + 关节点，
   进程自己以 0 退出 —— 不再「🏁 全部完成！」无限刷屏，Ctrl+C 也不会有 Traceback；
2. 每次到点先补一帧零速度，乌龟立刻停住，不会被上一帧的速度带着漂过去；
3. 加了单点超时 / 位姿超时看门狗：卡住了会明确报是哪一步卡、误差多大，
   而不是无声地在原地转圈；
4. 每个点记录「目标 / 实际 / 误差 / 用时」，结束时打一张汇总表，
   任务二截图 4（误差汇总）直接截这一段就行。

评审要看的那行日志保持原样：
    到达球 1，实际坐标 (2.05, 3.02)，误差 0.06
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from ball_pos_pkg.msg import BallPos


class BallNavigator(Node):
    # ---------------- 可调参数：想改就改这里 ----------------
    TARGET_ORDER = (1, 2, 3)   # 必须 1 -> 2 -> 3，不能跳、不能倒
    CONTROL_HZ = 10.0          # 控制循环频率

    ARRIVE_TOL = 0.10          # 到点阈值（m）；考核要求 <= 0.5
    ANGLE_TOL = 0.10           # 方位角对准阈值（rad）；差得比它大就原地转

    K_DIST = 1.5               # 距离 -> 线速度 的比例系数（越大越早达到限速）
    K_ANGLE = 1.0              # 方位差 -> 角速度 的比例系数
    V_MAX = 0.80               # 最大线速度（m/s）；再大也不会更快，转身才是瓶颈
    V_MIN = 0.08               # 最小线速度，别让乌龟最后几厘米爬不动
    W_MAX = 1.50               # 最大角速度（rad/s）

    POINT_TIMEOUT = 40.0       # 单个目标点最长允许耗时（s），超时就跳过
    POSE_TIMEOUT = 5.0         # /turtle1/pose 多久没更新就暂停发速度（s）

    def __init__(self):
        super().__init__('ball_navigator')

        # 状态
        self.pose = None            # 最新一帧乌龟位姿
        self.pose_stamp = None      # 上面那帧的时间戳
        self.balls = {}             # {id: (x, y)}
        self.nav_started = False
        self.current_idx = 0
        self.point_start = None     # 当前目标点的开始时间
        self.results = []           # 每个点的结果，结束时汇总
        self.done = False
        self._pose_warned = False
        self._shutdown_timer = None

        # 订阅 / 发布
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.ball_sub = self.create_subscription(
            BallPos, '/ball_pos', self.ball_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        self.control_timer = self.create_timer(
            1.0 / self.CONTROL_HZ, self.navigate)

        self.get_logger().info(
            '导航节点已启动：等待 /turtle1/pose 和 3 个球坐标 …')

    # ---------------- 回调 ----------------
    def pose_callback(self, msg):
        self.pose = msg
        self.pose_stamp = self.get_clock().now()

    def ball_callback(self, msg):
        # 发布端 2 Hz 反复发同一组坐标，用字典覆盖就行
        self.balls[int(msg.id)] = (float(msg.x), float(msg.y))

        if not self.nav_started and all(i in self.balls for i in self.TARGET_ORDER):
            self.nav_started = True
            self.point_start = self.get_clock().now()
            self.get_logger().info('✅ 已收齐 3 个球，开始按 1 -> 2 -> 3 导航')

    # ---------------- 工具函数 ----------------
    @staticmethod
    def normalize_angle(angle):
        """把角度归一化到 [-pi, pi]，避免 atan2 算出的差值绕圈。"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _stop(self):
        """发一帧零速度：线速度、角速度都归零。"""
        try:
            self.cmd_pub.publish(Twist())
        except Exception:              # 节点已在关闭过程中时忽略，避免多打一条报错
            pass

    # ---------------- 主控制循环（10 Hz 定时器回调） ----------------
    def navigate(self):
        if self.done or not self.nav_started or self.pose is None:
            return

        now = self.get_clock().now()

        # 看门狗 1：位姿话题断了就停手，别拿着旧位姿继续算
        if self.pose_stamp is not None:
            pose_age = (now - self.pose_stamp).nanoseconds / 1e9
            if pose_age > self.POSE_TIMEOUT:
                if not self._pose_warned:
                    self._pose_warned = True
                    self._stop()
                    self.get_logger().warn(
                        f'/turtle1/pose 已经 {pose_age:.1f} s 没有更新，'
                        '暂停发送速度指令（检查 turtlesim 是否还在跑）')
                return
        self._pose_warned = False

        # 三个点全走完
        if self.current_idx >= len(self.TARGET_ORDER):
            self._finish()
            return

        target_id = self.TARGET_ORDER[self.current_idx]
        if target_id not in self.balls:
            return

        if self.point_start is None:
            self.point_start = now

        # 看门狗 2：单个点超时，跳过并如实记进汇总
        elapsed = (now - self.point_start).nanoseconds / 1e9
        if elapsed > self.POINT_TIMEOUT:
            x, y = self.pose.x, self.pose.y
            tx, ty = self.balls[target_id]
            err = math.hypot(tx - x, ty - y)
            self.get_logger().warn(
                f'⚠️ 球 {target_id} 超过 {self.POINT_TIMEOUT:.0f} s 仍未到达，'
                f'当前误差 {err:.2f}，跳过该点')
            self._stop()
            self._record(target_id, x, y, err, elapsed, reached=False)
            self._advance()
            return

        # 正常闭环控制
        tx, ty = self.balls[target_id]
        x, y, theta = self.pose.x, self.pose.y, self.pose.theta

        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        angle_diff = self.normalize_angle(math.atan2(dy, dx) - theta)

        # 到达判定
        if dist <= self.ARRIVE_TOL:
            self._stop()               # 先刹车，再记结果
            self.get_logger().info(
                f'到达球 {target_id}，实际坐标 ({x:.2f}, {y:.2f})，误差 {dist:.2f}')
            self._record(target_id, x, y, dist, elapsed, reached=True)
            self._advance()
            return

        vel = Twist()
        if abs(angle_diff) > self.ANGLE_TOL:
            # 第二步：方位差太大 -> 先原地对准，不要边走边拐（会画大圈）
            vel.linear.x = 0.0
            vel.angular.z = self._clamp(self.K_ANGLE * angle_diff, self.W_MAX)
        else:
            # 第三步：对准了 -> 边走边微调，越近越慢，避免冲过头
            speed = max(self.V_MIN, min(self.V_MAX, self.K_DIST * dist))
            vel.linear.x = speed
            vel.angular.z = self._clamp(
                0.5 * self.K_ANGLE * angle_diff, self.W_MAX)

        self.cmd_pub.publish(vel)

    # ---------------- 状态推进 / 收尾 ----------------
    def _record(self, target_id, x, y, err, elapsed, reached):
        tx, ty = self.balls[target_id]
        self.results.append({
            'id': target_id,
            'tx': tx, 'ty': ty,
            'x': x, 'y': y,
            'err': err,
            'elapsed': elapsed,
            'reached': reached,
        })

    def _advance(self):
        self.current_idx += 1
        self.point_start = self.get_clock().now()
        if self.current_idx >= len(self.TARGET_ORDER):
            self._finish()

    def _finish(self):
        """收尾：刹车 -> 停定时器 -> 打一次汇总 -> 稍后关闭节点。"""
        if self.done:
            return
        self.done = True

        self._stop()
        if self.control_timer is not None:
            self.control_timer.cancel()

        self._log_summary()

        # 留 0.5 s 让日志刷完，再自己退出；不再等用户按 Ctrl+C
        self._shutdown_timer = self.create_timer(0.5, self._shutdown)

    def _log_summary(self):
        line = '-' * 58
        self.get_logger().info(line)
        self.get_logger().info('任务二 · 三球导航结果汇总')
        self.get_logger().info(line)

        max_err = 0.0
        all_ok = True
        for item in self.results:
            flag = 'OK ' if item['reached'] else '超时'
            self.get_logger().info(
                f"球 {item['id']}：目标 ({item['tx']:.2f}, {item['ty']:.2f})  "
                f"实际 ({item['x']:.2f}, {item['y']:.2f})  "
                f"误差 {item['err']:.2f}  用时 {item['elapsed']:.1f} s  [{flag}]")
            max_err = max(max_err, item['err'])
            if not item['reached']:
                all_ok = False

        verdict = 'PASS' if (all_ok and max_err <= 0.50) else 'FAIL'
        self.get_logger().info(
            f'最大误差 {max_err:.2f}（考核要求 <= 0.50）：{verdict}')
        self.get_logger().info(line)
        self.get_logger().info('🏁 全部完成，节点将在 0.5 s 后自动退出。')

    def _shutdown(self):
        """一次性定时器回调：干净地关掉本节点。"""
        if self._shutdown_timer is not None:
            self._shutdown_timer.cancel()
            self._shutdown_timer = None
        self.get_logger().info('导航节点已退出。')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = BallNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # 到点自动退出、或用户 Ctrl+C，都走这里：不打印 Traceback
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
