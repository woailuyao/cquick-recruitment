#!/usr/bin/env python3
"""任务2：订阅 /ball_pos 收集三个球，按 1->2->3 依次把乌龟开过去。
闭环：订阅 /turtle1/pose 实时读位姿，比例控制转向 + 前进。"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_tracker.msg import BallPos

# 到点判定：离目标小于这个距离就算到达（细则要求误差 <= 0.5，这里用 0.1 更保险）
ARRIVE_TOL = 0.1
# 允许前进时残留的角度偏差（弧度），超过它就先原地转
ANGLE_TOL = 0.15
# 转向角速度、前进线速度
TURN_SPEED = 1.2
MOVE_SPEED = 1.2
# 球的访问顺序
ORDER = [1, 2, 3]


class GoToBalls(Node):

    def __init__(self):
        super().__init__('go_to_balls')
        # 从 /ball_pos 收到的球坐标，key 是球 id
        self.balls = {}
        # 乌龟当前位姿（闭环的关键：不知道自己在哪就没法纠偏）
        self.pose = None
        # 当前去第几个目标（0/1/2）
        self.index = 0

        self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 10 Hz 控制频率：够顺滑，也不吃 CPU
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('等待 /ball_pos 发来三个球的坐标...')

    def on_ball(self, msg):
        """收到球坐标就存起来，同一个 id 只记第一次"""
        if msg.id not in self.balls:
            self.balls[msg.id] = (msg.x, msg.y)
            self.get_logger().info(
                f'记录球{msg.id} ({msg.x:.2f}, {msg.y:.2f})，已有 {len(self.balls)}/3')
            if len(self.balls) == 3:
                self.get_logger().info('三个球齐了，出发前往球1！')

    def on_pose(self, msg):
        """乌龟位姿，约每 0.1 秒来一次，存起来给控制循环用"""
        self.pose = msg

    def stop(self):
        """发一条零速度，让乌龟停下"""
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        # 数据没齐就什么都不做
        if self.pose is None or len(self.balls) < 3:
            return
        if self.index >= len(ORDER):
            return
        target = self.balls.get(ORDER[self.index])
        if target is None:
            return

        dx = target[0] - self.pose.x
        dy = target[1] - self.pose.y
        distance = math.hypot(dx, dy)

        if distance < ARRIVE_TOL:
            self.stop()
            self.get_logger().info(
                f'到达球{ORDER[self.index]}  位置=({self.pose.x:.2f}, {self.pose.y:.2f})'
                f'  误差={distance:.3f}')
            self.index += 1
            if self.index >= len(ORDER):
                self.get_logger().info('三个球全部走完，任务2 完成！')
            else:
                self.get_logger().info(f'前往球{ORDER[self.index]}')
            return

        # 目标相对乌龟的方位角 - 乌龟当前朝向 = 还要转多少
        angle_to_target = math.atan2(dy, dx)
        diff = angle_to_target - self.pose.theta
        # 折到 -pi ~ pi，避免乌龟绕远路转一大圈
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        cmd = Twist()
        if abs(diff) > ANGLE_TOL:
            # 方向没对正：原地转，不前进
            cmd.angular.z = TURN_SPEED if diff > 0 else -TURN_SPEED
        else:
            # 方向对正：前进，角度偏差越大走得越慢（比例控制）
            cmd.linear.x = MOVE_SPEED * max(0.3, 1.0 - abs(diff))
            cmd.angular.z = 1.5 * diff
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToBalls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
