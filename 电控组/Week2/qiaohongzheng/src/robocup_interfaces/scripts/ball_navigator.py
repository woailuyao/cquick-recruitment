#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from robocup_interfaces.msg import BallPos
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class BallNavigator(Node):

    def __init__(self):
        super().__init__('ball_navigator')

        # 发布速度指令
        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # 订阅三个球的坐标
        self.ball_sub = self.create_subscription(
            BallPos,
            '/ball_pos',
            self.ball_callback,
            10
        )

        # 订阅乌龟实时位置
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # 保存收到的球坐标
        self.balls = {}

        # 当前乌龟位置
        self.pose = None

        # 依次导航的球编号
        self.target_ids = [1, 2, 3]
        self.current_index = 0

        self.finished = False

        # 每隔 0.05 秒执行一次控制
        self.timer = self.create_timer(
            0.05,
            self.navigate
        )

        self.get_logger().info('导航节点已启动，等待三个球坐标')

    def ball_callback(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)

    def pose_callback(self, msg):
        self.pose = msg

    def navigate(self):

        if self.finished:
            return

        # 等待收齐三个球
        if len(self.balls) < 3 or self.pose is None:
            return

        # 检查是否已经完成三个目标
        if self.current_index >= 3:
            self.stop()
            self.finished = True
            self.get_logger().info('三个球全部到达，导航完成！')
            return

        # 获取当前目标
        ball_id = self.target_ids[self.current_index]
        target_x, target_y = self.balls[ball_id]

        # 获取乌龟当前位置
        x = self.pose.x
        y = self.pose.y
        theta = self.pose.theta

        # 计算目标距离
        dx = target_x - x
        dy = target_y - y

        distance = math.sqrt(dx**2 + dy**2)

        # 计算目标方位角
        target_angle = math.atan2(dy, dx)

        # 计算角度误差，并归一化到 [-pi, pi]
        angle_error = math.atan2(
            math.sin(target_angle - theta),
            math.cos(target_angle - theta)
        )

        msg = Twist()

        # 到达目标点
        if distance < 0.25:

            self.stop()

            self.get_logger().info(
                f'到达球 {ball_id}，'
                f'实际坐标 ({x:.2f}, {y:.2f})，'
                f'误差 {distance:.3f}'
            )

            self.current_index += 1
            return

        # 先调整方向
        if abs(angle_error) > 0.15:

            msg.linear.x = 0.0
            msg.angular.z = max(
                -1.5,
                min(1.5, 2.0 * angle_error)
            )

        # 对准之后向目标前进
        else:

            msg.linear.x = min(1.0, 0.8 * distance)

            msg.angular.z = max(
                -1.5,
                min(1.5, 3.0 * angle_error)
            )

        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):

    rclpy.init(args=args)

    node = BallNavigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
