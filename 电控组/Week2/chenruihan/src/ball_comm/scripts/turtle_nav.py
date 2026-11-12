#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ball_comm.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleNav(Node):
    def __init__(self):
        super().__init__('turtle_nav')
        self.balls = {}          # id -> (x, y)
        self.waypoints = []      # 按 id 排好的 3 个目标点
        self.idx = 0             # 当前目标索引
        self.pose = None         # 最新位姿
        self.done = False

        self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_cb)
        self.get_logger().info('导航节点启动，等待 3 个球')

    def ball_cb(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        if len(self.balls) == 3 and not self.waypoints:
            self.waypoints = [self.balls[i] for i in sorted(self.balls)]
            self.get_logger().info(
                f'收齐 3 个球，航点: {self.waypoints}，开始导航')

    def pose_cb(self, msg):
        self.pose = msg

    def stop(self):
        self.pub.publish(Twist())

    def control_cb(self):
        if self.done or not self.waypoints or self.pose is None:
            return
        if self.idx >= len(self.waypoints):
            self.stop()
            self.done = True
            self.get_logger().info('三个球全部到达，任务完成')
            return

        tx, ty = self.waypoints[self.idx]
        x, y, theta = self.pose.x, self.pose.y, self.pose.theta
        d = math.hypot(tx - x, ty - y)
        a = math.atan2(ty - y, tx - x) - theta
        a = math.atan2(math.sin(a), math.cos(a))   # 归一化到 [-pi, pi]

        if d < 0.1:
            self.stop()
            self.get_logger().info(
                f'到达球 {self.idx + 1}，实际 ({x:.2f}, {y:.2f})，误差 {d:.3f}')
            self.idx += 1
            return

        cmd = Twist()
        if abs(a) > 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0 * a
        else:
            cmd.linear.x = min(0.5 * d, 1.0)
            cmd.angular.z = 1.0 * a
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
