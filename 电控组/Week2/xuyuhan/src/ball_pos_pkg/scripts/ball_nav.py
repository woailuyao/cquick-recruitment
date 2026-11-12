#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose          # 注意：不是 geometry_msgs/Pose
from ball_pos_pkg.msg import BallPos

ORDER = [1, 2, 3]                        # 必须按 1 → 2 → 3 的顺序


class BallNav(Node):
    def __init__(self):
        super().__init__('ball_nav')

        # 参数：方便调，调参不用改代码
        self.declare_parameter('max_linear', 1.2)     # 最大线速度 m/s
        self.declare_parameter('max_angular', 2.0)    # 最大角速度 rad/s
        self.declare_parameter('dist_tol', 0.30)      # 到点阈值（要求 ≤0.5）
        self.declare_parameter('angle_tol', 0.15)     # 对准阈值 rad

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.timer = self.create_timer(0.05, self.control)    # 20 Hz 控制循环

        self.balls = {}          # id -> (x, y)
        self.pose = None         # 当前位姿
        self.target_idx = 0      # 正在去第几个球
        self.done = False

    # ---------- 回调：只负责收数据 ----------
    def on_ball(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)

    def on_pose(self, msg):
        self.pose = msg

    # ---------- 工具 ----------
    @staticmethod
    def normalize(a):
        """把角度折算到 [-pi, pi]"""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    # ---------- 控制循环 ----------
    def control(self):
        if self.pose is None:
            return

        # 收齐 3 个球才开始动
        if len(self.balls) < 3:
            return

        # 全部到齐，停车
        if self.target_idx >= len(ORDER):
            if not self.done:
                self.done = True
                self.get_logger().info('三个球全部到达，任务结束')
                self.cmd_pub.publish(Twist())
            return

        max_v = self.get_parameter('max_linear').value
        max_w = self.get_parameter('max_angular').value
        dist_tol = self.get_parameter('dist_tol').value
        angle_tol = self.get_parameter('angle_tol').value

        bid = ORDER[self.target_idx]
        tx, ty = self.balls[bid]
        dx, dy = tx - self.pose.x, ty - self.pose.y
        dist = math.hypot(dx, dy)

        # 到点：打印误差，切下一个
        if dist < dist_tol:
            self.get_logger().info(
                f'到达球{bid}，实际({self.pose.x:.2f}, {self.pose.y:.2f})，'
                f'目标({tx:.2f}, {ty:.2f})，误差 {dist:.3f}')
            self.target_idx += 1
            self.cmd_pub.publish(Twist())      # 先停一下，避免冲过头
            return

        err = self.normalize(math.atan2(dy, dx) - self.pose.theta)

        cmd = Twist()
        if abs(err) > angle_tol:
            # 没对准：原地转
            cmd.linear.x = 0.0
            cmd.angular.z = self.clamp(1.5 * err, -max_w, max_w)
        else:
            # 对准了：边走边微调，越近越慢
            cmd.linear.x = min(max_v, 1.2 * dist)
            cmd.angular.z = self.clamp(1.5 * err, -max_w, max_w)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BallNav()
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