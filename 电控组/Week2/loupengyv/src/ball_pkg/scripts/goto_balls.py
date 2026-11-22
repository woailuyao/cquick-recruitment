#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pkg.msg import BallPos

# 顺序和坐标都是细则钉死的
BALLS = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]

ARRIVE_DIST = 0.15    # 离目标多近算"到了"
ALIGN_ANGLE = 0.25    # 角度差大于这个值 -> 先原地转


class GotoBalls(Node):
    def __init__(self):
        super().__init__('goto_balls')
        self.pose = None        # 乌龟当前位置
        self.balls = {}         # 收到的球 {id: (x, y)}
        self.idx = 0            # 当前去第几个（0/1/2）
        self.done = False
        self.started = False

        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_timer(0.1, self.control)     # 10 Hz 控制循环
        self.get_logger().info('goto_balls 启动，等待 3 个球的位置...')

    def on_pose(self, msg):
        self.pose = msg

    def on_ball(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        if len(self.balls) == 3 and not self.started:
            self.started = True
            self.get_logger().info('收齐 3 个球，开始导航')

    def control(self):
        # 没收齐球、或者还没收到位姿，就什么都不做
        if self.done or self.pose is None or len(self.balls) < 3:
            return

        tid, tx, ty = BALLS[self.idx]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)

        # 目标方向与当前朝向的夹角，归一化到 [-pi, pi]
        target = math.atan2(dy, dx)
        diff = math.atan2(math.sin(target - self.pose.theta),
                          math.cos(target - self.pose.theta))

        cmd = Twist()

        if dist < ARRIVE_DIST:
            # 到点了，记一行日志然后换下一个目标
            self.get_logger().info(
                '到达第 %d 个球，实际坐标 (%.2f, %.2f)，误差 %.2f'
                % (tid, self.pose.x, self.pose.y, dist))
            self.idx += 1
            if self.idx >= len(BALLS):
                self.done = True
                self.get_logger().info('三个球全部到达，导航结束')
        elif abs(diff) > ALIGN_ANGLE:
            # 方向差太多：原地转，不往前走
            cmd.linear.x = 0.0
            cmd.angular.z = max(-2.0, min(2.0, 2.0 * diff))
        else:
            # 对准了：边走边微调，越近越慢，避免冲过头
            cmd.linear.x = max(0.15, min(1.2, 1.0 * dist))
            cmd.angular.z = max(-1.5, min(1.5, 3.0 * diff))

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = GotoBalls()
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
