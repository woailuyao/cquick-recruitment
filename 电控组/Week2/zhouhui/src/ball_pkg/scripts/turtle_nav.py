#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pkg.msg import BallPos

class BallNavigator(Node):
    def __init__(self):
        super().__init__('ball_navigator')
        self.pose = None
        self.balls = {}
        self.order = [1, 2, 3]   # 强制顺序
        self.index = 0
        self.done = False

        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 控制频率 10Hz
        self.create_timer(0.1, self.control_loop)

    def pose_cb(self, msg):
        self.pose = msg

    def ball_cb(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)

    def control_loop(self):
        if self.pose is None or self.done:
            return
        if len(self.balls) < 3:  # 没收齐 3 个球，继续等
            return
        if self.index >= len(self.order):
            self.done = True
            self.stop()
            self.get_logger().info('全部 3 个球到达，停止')
            return

        ball_id = self.order[self.index]
        goal_x, goal_y = self.balls[ball_id]

        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y
        distance = math.hypot(dx, dy)

        # 到达阈值：0.5
        if distance < 0.5:
            self.stop()
            self.get_logger().info(
                f'到达球 {ball_id}，实际坐标 ({self.pose.x:.3f}, {self.pose.y:.3f})，'
                f'目标 ({goal_x}, {goal_y})，误差 {distance:.3f}'
            )
            self.index += 1
            return

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.pose.theta)

        cmd = Twist()
        # 角度偏差大：原地转
        if abs(angle_error) > 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 * angle_error
        else:
            # 偏差小：边走边调，越近越慢
            cmd.linear.x = min(0.8, 1.0 * distance)
            cmd.angular.z = 1.5 * angle_error

        cmd.angular.z = max(-2.0, min(2.0, cmd.angular.z))
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = BallNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
