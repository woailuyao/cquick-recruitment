#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 2 · 导航节点
订阅 /ball_pos 收齐 3 个球，再用闭环控制让乌龟按 1 → 2 → 3 依次走过去。
控制律（每 50 ms 算一次）：
    方位角差大  -> linear.x = 0，只转不走（先对准）
    方位角对准  -> linear.x 跟距离挂钩（越近越慢，避免冲过头）
    距离 < 0.1  -> 算到达，打日志、切下一个点
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose

from ball_pos_pkg.msg import BallPos

ORDER = [1, 2, 3]      # 必须按这个顺序，不能跳、不能倒
TOL_DIST = 0.10        # 到点判定阈值（要求 ≤0.5，这里收紧到 0.1 留余量）
TOL_ANGLE = 0.15       # 方位角误差小于 ~8.6° 就算对准了
MAX_LINEAR = 0.8       # 最大线速度（m/s）
MAX_ANGULAR = 2.0      # 最大角速度（rad/s）
K_ANGLE = 2.5          # 角速度比例系数
K_DIST = 1.5           # 线速度比例系数：速度 = K_DIST × 距离


def norm(angle):
    """把角度折到 [-pi, pi]。不折的话 -179° 和 +179° 会被算成差 358°。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class BallNavigator(Node):
    def __init__(self):
        super().__init__("ball_navigator")

        self.balls = {}        # id -> (x, y)，用字典按 id 存
        self.pose = None       # 乌龟当前位姿 (x, y, theta)
        self.index = 0         # 正在去第几个球（ORDER 的下标）
        self.done = False
        self.started = False

        self.cmd_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.create_subscription(BallPos, "ball_pos", self.ball_cb, 10)
        self.create_subscription(Pose, "turtle1/pose", self.pose_cb, 10)
        self.timer = self.create_timer(0.05, self.control)  # 20 Hz 控制频率

        self.get_logger().info("导航节点启动，等待收齐 3 个球 ...")

    def ball_cb(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)

    def pose_cb(self, msg):
        self.pose = msg

    def control(self):
        # 没收齐 3 个球、或还没收到自己的位姿，就继续等
        if self.done or self.pose is None or len(self.balls) < 3:
            return
        if not self.started:
            self.started = True
            self.get_logger().info("已收齐 3 个球：%s" % sorted(self.balls.items()))

        ball_id = ORDER[self.index]
        tx, ty = self.balls[ball_id]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)                                # 到目标的距离
        err = norm(math.atan2(dy, dx) - self.pose.theta)          # 方位角误差

        cmd = Twist()
        if dist < TOL_DIST:
            # 到了：停车 → 打日志 → 切下一个
            self.cmd_pub.publish(cmd)  # 全 0 的 Twist = 停车
            self.get_logger().info(
                "到达第 %d 个球，实际坐标 (%.2f, %.2f)，目标 (%.2f, %.2f)，误差 %.3f"
                % (ball_id, self.pose.x, self.pose.y, tx, ty, dist)
            )
            self.index += 1
            if self.index >= len(ORDER):
                self.done = True
                self.get_logger().info("三个球全部到达，导航结束")
            return

        if abs(err) > TOL_ANGLE:
            cmd.linear.x = 0.0                                    # 先对准：只转不走
        else:
            cmd.linear.x = min(MAX_LINEAR, K_DIST * dist)         # 越近越慢
        cmd.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, K_ANGLE * err))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BallNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
