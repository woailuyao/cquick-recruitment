#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from week2_ball.msg import BallPos


class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')

        self.targets = {}          # id -> (x, y)，收球用
        self.order = [1, 2, 3]     # 按 1→2→3 顺序走
        self.index = 0             # 当前第几个
        self.pose = None           # 乌龟当前位姿

        self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.create_timer(0.1, self.control)     # 10 Hz 控制循环

    def on_ball(self, msg):
        self.targets[msg.id] = (msg.x, msg.y)

    def on_pose(self, msg):
        self.pose = msg
    
    def control(self):
        # 三个球没收齐、或不知道自己在哪 → 什么都不做
        if self.pose is None or len(self.targets) < 3:
            return
        if self.index >= len(self.order):
            return

        tx, ty = self.targets[self.order[self.index]]
        x, y, theta = self.pose.x, self.pose.y, self.pose.theta

        dist = math.hypot(tx - x, ty - y)
        angle_to = math.atan2(ty - y, tx - x)
        err = math.atan2(math.sin(angle_to - theta), math.cos(angle_to - theta))

        cmd = Twist()

        if dist < 0.15:                                   # 到点
            self.get_logger().info(
                f'到达第 {self.order[self.index]} 个球，实际坐标 ({x:.2f}, {y:.2f})，误差 {dist:.2f}')
            self.index += 1
            self.pub.publish(Twist())                      # 停下
            return

        if abs(err) > 0.25:                                # 没对准：原地转
            cmd.linear.x = 0.0
            cmd.angular.z = max(-2.0, min(2.0, 2.0 * err))
        else:                                              # 对准：边走边修
            cmd.linear.x = max(-1.5, min(1.5, 1.5 * dist))
            cmd.angular.z = max(-2.0, min(2.0, 1.5 * err))

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = NavNode()
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