#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
任务2 · 闭环导航节点（路线 A）

逻辑：
  1. 订阅 /ball_pos，用字典按 id 存坐标，收齐 3 个球才开始动；
  2. 订阅 /turtle1/pose 拿乌龟当前 (x, y, theta)；
  3. 每个控制周期算「目标角 alpha」和「距离 dist」：
       - |alpha| 大  -> 原地转（linear.x = 0），先对准；
       - |alpha| 小  -> 边走边微调，距离越近速度越小；
       - dist 小于阈值 -> 记为到达，打印实际坐标和误差，切下一个点；
  4. 3 个点走完停下来，撤销轨迹笔，退出。
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from cquick_ball.msg import BallPos

# 需要到达的球的顺序：1 -> 2 -> 3
ORDER = [1, 2, 3]


def normalize_angle(ang):
    """把角度规整到 [-pi, pi]"""
    while ang > math.pi:
        ang -= 2.0 * math.pi
    while ang < -math.pi:
        ang += 2.0 * math.pi
    return ang


class BallNavigator(Node):
    def __init__(self):
        super().__init__('ball_navigator')

        # ---- 参数 ----
        self.declare_parameter('goal_tolerance', 0.15)   # 到点阈值（<0.5 即可）
        self.declare_parameter('max_linear', 1.4)        # 最大线速度
        self.declare_parameter('max_angular', 2.6)       # 最大角速度
        self.declare_parameter('kp_linear', 1.8)         # 线速度比例系数
        self.declare_parameter('kp_angular', 3.0)        # 角速度比例系数
        self.declare_parameter('align_threshold', 0.35)  # 方位差超过它就原地转
        self.declare_parameter('approach_speed', 0.6)    # 边走边转时的最低线速度

        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.max_v = float(self.get_parameter('max_linear').value)
        self.max_w = float(self.get_parameter('max_angular').value)
        self.kp_v = float(self.get_parameter('kp_linear').value)
        self.kp_w = float(self.get_parameter('kp_angular').value)
        self.align_th = float(self.get_parameter('align_threshold').value)
        self.v_floor = float(self.get_parameter('approach_speed').value)

        # ---- 状态 ----
        self.balls = {}          # id -> (x, y)
        self.pose = None         # 当前位姿
        self.index = 0           # 当前在追第几个球（ORDER 的下标）
        self.done = False
        self.started = False

        # ---- 接口 ----
        self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.report_pub = self.create_publisher(BallPos, '/reached_ball', 10)

        self.create_timer(0.05, self.control_loop)  # 20 Hz 控制周期
        self.get_logger().info('导航节点已启动，等待收齐 3 个球 …')

    # ---------- 回调 ----------
    def ball_cb(self, msg):
        if not self.started:
            self.balls[msg.id] = (msg.x, msg.y)
            if len(self.balls) >= 3:
                self.started = True
                self.get_logger().info(
                    '已收齐 3 个球: ' + self._fmt_balls(self.balls) +
                    '，开始导航')
                self.get_logger().info(
                    f'目标顺序: 球1 -> 球2 -> 球3')

    def pose_cb(self, msg):
        self.pose = msg

    @staticmethod
    def _fmt_balls(balls):
        items = sorted(balls.items())
        return ', '.join(f'球{i}({x:.1f},{y:.1f})' for i, (x, y) in items)

    # ---------- 控制 ----------
    def control_loop(self):
        if self.done or not self.started or self.pose is None:
            return
        if self.index >= len(ORDER):
            self.finish()
            return

        ball_id = ORDER[self.index]
        if ball_id not in self.balls:
            return

        gx, gy = self.balls[ball_id]
        dx = gx - self.pose.x
        dy = gy - self.pose.y
        dist = math.hypot(dx, dy)
        alpha = normalize_angle(math.atan2(dy, dx) - self.pose.theta)

        cmd = Twist()

        if dist < self.goal_tol:
            # 到点：停车，记录
            self.cmd_pub.publish(Twist())  # 全零 = 停
            self.arrive(ball_id, gx, gy, dist)
            return

        # ---- 先对准再走 ----
        if abs(alpha) > self.align_th:
            # 方位差太大：原地转，不前进
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_w, min(self.max_w, self.kp_w * alpha))
        else:
            # 方位基本对上：边走边微调，越近越慢
            v = min(self.max_v, self.kp_v * dist)
            v = max(v, self.v_floor)
            # 转弯时稍微降速，避免画大弧
            v *= max(0.35, 1.0 - 0.6 * abs(alpha) / self.align_th)
            cmd.linear.x = v
            cmd.angular.z = max(-self.max_w, min(self.max_w, self.kp_w * alpha))

        self.cmd_pub.publish(cmd)

    def arrive(self, ball_id, gx, gy, dist):
        self.index += 1
        total = len(ORDER)
        self.get_logger().info(
            f'>>> 到达第 {self.index}/{total} 个球（球 {ball_id}），'
            f'实际坐标 ({self.pose.x:.3f}, {self.pose.y:.3f})，'
            f'目标坐标 ({gx:.1f}, {gy:.1f})，误差 d = {dist:.3f} m')

        # 把「已到达哪个球」也通过自定义消息发出去，方便用 ros2 topic echo 验证
        m = BallPos()
        m.id = ball_id
        m.x = self.pose.x
        m.y = self.pose.y
        self.report_pub.publish(m)

        if self.index >= len(ORDER):
            self.finish()

    def finish(self):
        if self.done:
            return
        self.done = True
        # 停车
        self.cmd_pub.publish(Twist())
        # 抬笔，让最后一段轨迹不再继续延伸
        self.set_pen(off=True)
        self.get_logger().info('全部 3 个点已到达，导航结束。')

    def set_pen(self, off=True):
        if not self.pen_client.service_is_ready():
            self.pen_client.wait_for_service(timeout_sec=2.0)
        req = SetPen.Request()
        req.r = 255
        req.g = 0
        req.b = 0
        req.width = 2
        req.off = 1 if off else 0
        self.pen_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
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
