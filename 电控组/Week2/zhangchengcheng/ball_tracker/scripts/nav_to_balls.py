#!/usr/bin/env python3
"""任务 2 · 闭环导航节点

订阅 /ball_pos 收齐 3 个球坐标，再订阅 /turtle1/pose 拿到乌龟当前位姿，
用 geometry_msgs/Twist 让乌龟按 1 -> 2 -> 3 的顺序依次走过去。

控制思路（闭环）：
  1. 算到目标点的距离 dist 和方位角误差 err_theta；
  2. |err_theta| 大于阈值 -> 原地转（linear.x = 0），先对准再走；
  3. 对准后 -> 边走边微调，线速度随距离衰减（越近越慢），避免冲过头；
  4. dist 小于阈值 -> 认为到达，打印实际坐标和误差，切下一个点。

依赖：先启动 turtlesim（ros2 run turtlesim turtlesim_node）和任务 1 的发布者。
节点启动并收齐坐标后，会自动调用一次服务 /clear 清屏，让轨迹看得清。
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ball_tracker.msg import BallPos
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.msg import Pose

# 必须按这个顺序走，不能跳、不能倒
ORDER = [1, 2, 3]


def normalize_angle(angle):
    """把角度归一化到 [-pi, pi]，避免 -179° 和 179° 被当成差了 358°"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, low, high):
    return max(low, min(high, value))


class BallNavigator(Node):
    def __init__(self):
        super().__init__('ball_navigator')

        # 控制参数（都可用 --ros-args -p xxx:=yyy 改）
        self.declare_parameter('dist_tolerance', 0.15)   # 到点判定阈值
        self.declare_parameter('angle_tolerance', 0.05)  # 对准判定阈值(rad)
        self.declare_parameter('k_linear', 0.9)          # 距离 -> 线速度 的比例
        self.declare_parameter('k_angular', 2.0)         # 角度误差 -> 角速度 的比例
        self.declare_parameter('max_linear', 1.6)        # 线速度上限，防止冲过头
        self.declare_parameter('min_linear', 0.18)       # 线速度下限，防止磨蹭不到位
        self.declare_parameter('max_angular', 1.8)       # 角速度上限

        self.balls = {}        # {id: (x, y)}
        self.targets = []      # 按 ORDER 排好的目标点
        self.idx = 0
        self.pose = None
        self.errors = {}       # {id: 最终误差}
        self.finished = False

        self.sub_ball = self.create_subscription(
            BallPos, 'ball_pos', self.ball_callback, 10)
        self.sub_pose = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cli_clear = self.create_client(Empty, '/clear')

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz 控制
        self.get_logger().info('导航节点已启动，等待 /ball_pos 上的 3 个球坐标...')

    # ---------- 回调 ----------
    def ball_callback(self, msg):
        if msg.id not in ORDER:
            return
        self.balls[msg.id] = (msg.x, msg.y)
        # 收齐 3 个才动身；没收齐继续等
        if len(self.balls) == 3 and not self.targets:
            self.targets = [self.balls[i] for i in ORDER]
            self.get_logger().info(
                '3 个球坐标收齐：' +
                ' | '.join(f'球{i}({self.balls[i][0]:.1f},{self.balls[i][1]:.1f})'
                           for i in ORDER))
            self.get_logger().info('开始依次前往：球 1 -> 球 2 -> 球 3')
            self.clear_screen()

    def pose_callback(self, msg):
        self.pose = msg

    def clear_screen(self):
        """调用服务 /clear（std_srvs/srv/Empty）清掉旧轨迹"""
        if not self.cli_clear.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/clear 服务不可用，turtlesim 没开？')
            return
        self.cli_clear.call_async(Empty.Request())
        self.get_logger().info('已调用服务 /clear (std_srvs/srv/Empty) 清屏')

    # ---------- 控制 ----------
    def stop(self):
        self.pub_cmd.publish(Twist())

    def control_loop(self):
        if self.finished or not self.targets or self.pose is None:
            return

        ball_id = ORDER[self.idx]
        tx, ty = self.targets[self.idx]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)

        dist_tol = float(self.get_parameter('dist_tolerance').value)
        ang_tol = float(self.get_parameter('angle_tolerance').value)

        # 到了：记录误差，切下一个点
        if dist <= dist_tol:
            self.errors[ball_id] = dist
            self.get_logger().info(
                f'到达球 {ball_id}：实际坐标 ({self.pose.x:.2f}, {self.pose.y:.2f})  '
                f'目标 ({tx:.2f}, {ty:.2f})  误差 {dist:.3f}')
            self.stop()
            self.idx += 1
            if self.idx >= len(ORDER):
                self.finished = True
                self.timer.cancel()
                self.get_logger().info('===== 三个点的最终误差汇总 =====')
                for bid in ORDER:
                    self.get_logger().info(
                        f'  球 {bid}：目标 {self.balls[bid]}，最终误差 {self.errors[bid]:.3f}')
                self.get_logger().info('三点全部走完，任务结束（Ctrl+C 退出）')
            else:
                self.get_logger().info(f'下一个目标：球 {ORDER[self.idx]}')
            return

        # 没到：算方位角误差
        err_theta = normalize_angle(math.atan2(dy, dx) - self.pose.theta)

        twist = Twist()
        if abs(err_theta) > ang_tol:
            # 先对准：只转不走
            twist.linear.x = 0.0
            twist.angular.z = clamp(
                float(self.get_parameter('k_angular').value) * err_theta,
                -float(self.get_parameter('max_angular').value),
                float(self.get_parameter('max_angular').value))
        else:
            # 对准了：边走边微调，速度跟距离挂钩（越近越慢）
            v = float(self.get_parameter('k_linear').value) * dist
            twist.linear.x = clamp(
                v,
                float(self.get_parameter('min_linear').value),
                float(self.get_parameter('max_linear').value))
            twist.angular.z = float(self.get_parameter('k_angular').value) * 0.5 * err_theta

        self.pub_cmd.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BallNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
