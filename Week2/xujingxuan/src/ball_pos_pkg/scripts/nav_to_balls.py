#!/usr/bin/env python3
"""任务2：订阅 ball_pos，用 Twist 让乌龟按 1→2→3 依次走到三个坐标。"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from ball_pos_pkg.msg import BallPos
from turtlesim.msg import Pose

GOAL_TOL = 0.15   # 距离阈值：小于它就算到了
ANG_TOL = 0.25    # 方位角阈值：超过它先原地转
K_LIN = 1.5       # 线速度比例系数
K_ANG = 2.0       # 角速度比例系数
V_MAX = 1.2       # 线速度上限
W_MAX = 2.0       # 角速度上限


class NavToBalls(Node):
    def __init__(self):
        super().__init__('nav_to_balls')

        self.balls = {}          # id -> (x, y)
        self.pose = None         # 乌龟当前位姿
        self.idx = 0             # 当前要去第几个球（0 表示球1）
        self.finished = False

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.timer = self.create_timer(0.1, self.control)

        self.get_logger().info('导航节点已启动，等待收齐 3 个球的坐标 ...')

    def on_ball(self, msg):
        if msg.id not in self.balls:
            self.balls[msg.id] = (msg.x, msg.y)
            self.get_logger().info(
                f'收到 球{msg.id} ({msg.x:.2f}, {msg.y:.2f})  [{len(self.balls)}/3]'
            )

    def on_pose(self, msg):
        self.pose = msg

    def control(self):
        if self.finished or self.pose is None or len(self.balls) < 3:
            return

        gx, gy = self.balls[self.idx + 1]
        dx, dy = gx - self.pose.x, gy - self.pose.y
        d = math.hypot(dx, dy)

        e = math.atan2(dy, dx) - self.pose.theta
        e = math.atan2(math.sin(e), math.cos(e))

        tw = Twist()

        if d <= GOAL_TOL:
            # 到点：先发一条全 0 的 Twist 刹车，再切下一个目标
            self.pub.publish(tw)
            self.get_logger().info(
                f'到达球{self.idx + 1}，实际坐标 ({self.pose.x:.3f}, {self.pose.y:.3f})，'
                f'目标 ({gx:.1f}, {gy:.1f})，误差 {d:.3f}'
            )
            self.idx += 1
            if self.idx >= 3:
                self.get_logger().info('三个点全部到达，任务完成')
                self.finished = True
            return

        if abs(e) > ANG_TOL:
            tw.linear.x = 0.0                                  # 先原地对准，不走
            tw.angular.z = max(-W_MAX, min(W_MAX, K_ANG * e))
        else:
            tw.linear.x = min(V_MAX, K_LIN * d)                # 越近越慢
            tw.angular.z = max(-W_MAX, min(W_MAX, K_ANG * e))

        self.pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
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
