#!/usr/bin/env python3
"""任务 2 · 订阅 /ball_pos，收齐 3 个球后按 1 -> 2 -> 3 的顺序依次走过去。

路线选择：闭环跟随（路线 A）
    - 订阅 /turtle1/pose 拿到乌龟当前 (x, y, theta)；
    - 每个控制周期（20 Hz）算一次「到目标点的距离、和目标方向的夹角」；
    - 夹角大 -> 原地转（linear.x = 0）；夹角小 -> 边走边微调，越近越慢；
    - 距离小于阈值就认为到达，日志记录实际坐标和误差，切下一个点。

运行（先在另一个终端跑任务 1 的发布者，否则收不到球坐标）：
    source ~/ros2_ws/install/setup.bash
    python3 src/ball_navigator.py
可选参数：
    python3 src/ball_navigator.py --ros-args -p max_linear:=0.6 -p goal_tolerance:=0.10
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose

from ball_comm.msg import BallPos

# 必须按 1 -> 2 -> 3 的顺序
ORDER = [1, 2, 3]


def normalize_angle(angle):
    """把角度归一化到 (-pi, pi]，避免 -3.1 和 +3.1 被当成差很多。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class BallNavigator(Node):
    """收齐 3 个球坐标，然后依次走到每个球的位置。"""

    def __init__(self):
        super().__init__('ball_navigator')

        # 可调参数：速度给太大就会冲过头，现场调这三个最有用
        self.declare_parameter('max_linear', 0.8)        # 最大线速度 m/s
        self.declare_parameter('max_angular', 2.0)       # 最大角速度 rad/s
        self.declare_parameter('goal_tolerance', 0.10)   # 到点阈值（细则要求 <= 0.5）
        self.declare_parameter('rotate_threshold', 0.30)  # 夹角超过它就原地转
        self.declare_parameter('linear_gain', 1.5)       # 线速度 = 增益 * 距离
        self.declare_parameter('angular_gain', 4.0)      # 角速度 = 增益 * 夹角

        self.balls = {}    # id -> (x, y)
        self.pose = None   # 乌龟当前位姿
        self.index = 0     # 正在前往 ORDER 里的第几个
        self.done = False
        self.results = []

        self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)
        self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # 控制循环 20 Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('ball_navigator 已启动，等待收齐 3 个球 ...')

    # ---------------- 回调 ----------------
    def ball_callback(self, msg):
        if msg.id in self.balls:
            return  # 发布者一直发，同一个球只存一次

        self.balls[msg.id] = (msg.x, msg.y)
        self.get_logger().info(
            '收到球 %d：(%.2f, %.2f)，已收 %d/3' % (msg.id, msg.x, msg.y, len(self.balls))
        )

        if len(self.balls) == len(ORDER):
            self.get_logger().info('3 个球已收齐，开始按 1 -> 2 -> 3 依次前往')
            self.announce_target()

    def pose_callback(self, msg):
        self.pose = msg

    def announce_target(self):
        ball_id = ORDER[self.index]
        x, y = self.balls[ball_id]
        self.get_logger().info('前往球 %d：目标 (%.2f, %.2f)' % (ball_id, x, y))

    # ---------------- 控制 ----------------
    def control_loop(self):
        # 没收齐不动；没有位姿不动；跑完了不动
        if self.done or self.pose is None or len(self.balls) < len(ORDER):
            return
        if self.index >= len(ORDER):
            return

        ball_id = ORDER[self.index]
        target_x, target_y = self.balls[ball_id]

        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.hypot(dx, dy)
        # 目标方向 - 当前朝向 = 还差多少角度
        angle_error = normalize_angle(math.atan2(dy, dx) - self.pose.theta)

        max_linear = self.get_parameter('max_linear').value
        max_angular = self.get_parameter('max_angular').value
        tolerance = self.get_parameter('goal_tolerance').value
        rotate_threshold = self.get_parameter('rotate_threshold').value
        linear_gain = self.get_parameter('linear_gain').value
        angular_gain = self.get_parameter('angular_gain').value

        # 到点判定：距离够近就算到了
        if distance <= tolerance:
            self.finish_point(ball_id, target_x, target_y, distance)
            return

        cmd = Twist()
        if abs(angle_error) > rotate_threshold:
            # 方向还没对上：原地转，先对准再走
            cmd.linear.x = 0.0
            cmd.angular.z = self.clamp(angular_gain * angle_error, max_angular)
        else:
            # 方向对上了：边走边微调，越接近目标速度越小
            cmd.linear.x = min(max_linear, linear_gain * distance)
            cmd.angular.z = self.clamp(angular_gain * angle_error, max_angular)

        self.cmd_pub.publish(cmd)

    def finish_point(self, ball_id, target_x, target_y, distance):
        """到达一个点：停车、记日志、切下一个点。"""
        self.cmd_pub.publish(Twist())  # 速度归零，否则乌龟会一直冲

        actual_x = self.pose.x
        actual_y = self.pose.y
        self.results.append((ball_id, target_x, target_y, actual_x, actual_y, distance))
        self.get_logger().info(
            '到达第 %d 个球，实际坐标 (%.2f, %.2f)，误差 %.3f'
            % (ball_id, actual_x, actual_y, distance)
        )

        self.index += 1
        if self.index >= len(ORDER):
            self.done = True
            self.print_summary()
        else:
            self.announce_target()

    def print_summary(self):
        self.get_logger().info('=== 三个点全部到达，误差汇总 ===')
        for ball_id, tx, ty, ax, ay, err in self.results:
            self.get_logger().info(
                '球 %d：目标 (%.2f, %.2f) -> 实际 (%.2f, %.2f)，误差 %.3f'
                % (ball_id, tx, ty, ax, ay, err)
            )

    @staticmethod
    def clamp(value, limit):
        """把值限制在 [-limit, limit] 内。"""
        return max(-limit, min(limit, value))


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


if __name__ == '__main__':
    main()
