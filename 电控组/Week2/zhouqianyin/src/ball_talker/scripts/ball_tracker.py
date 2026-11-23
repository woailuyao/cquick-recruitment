#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import atan2, sqrt
from ball_talker.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker')
        self.ball_sub = self.create_subscription(BallPos, '/ball_pos', self.ball_callback, 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.ball_dict = dict()   # 按id缓存小球坐标
        self.order = [1,2,3]
        self.idx = 0
        self.threshold = 0.5
        self.start_flag = False   # 是否集齐3个，允许出发
        self.finished = False

        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.turtle_theta = 0.0

    def ball_callback(self, msg):
        if self.start_flag or self.finished:
            return
        # 存入字典，id作为key
        self.ball_dict[msg.id] = (msg.x, msg.y)
        self.get_logger().info(f"收到球 id={msg.id} x={msg.x:.2f}, y={msg.y:.2f}")
        # 集齐3个id，解锁导航
        if len(self.ball_dict) == 3:
            self.start_flag = True
            self.get_logger().info("✅ 三个小球信息全部接收完成，开始导航！")

    def pose_callback(self, msg):
        if not self.start_flag or self.finished:
            return
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta

        if self.idx >= len(self.order):
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.get_logger().info("🎉 全部3个点走完！任务结束")
            self.finished = True
            return

        tid = self.order[self.idx]
        tx, ty = self.ball_dict[tid]
        dx = tx - self.turtle_x
        dy = ty - self.turtle_y
        dist = sqrt(dx*dx + dy*dy)

        if dist < self.threshold:
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.get_logger().info(f"✅ 到达球 {tid}, 实际坐标({self.turtle_x:.2f},{self.turtle_y:.2f}), 误差d={dist:.2f}")
            self.idx += 1
            return

        angle_to_target = atan2(dy, dx)
        angle_err = angle_to_target - self.turtle_theta
        while angle_err > 3.14159:
            angle_err -= 2 * 3.14159
        while angle_err < -3.14159:
            angle_err += 2 * 3.14159

        vel = Twist()
        vel.linear.x = 0.4
        vel.angular.z = 0.6 * angle_err
        self.cmd_pub.publish(vel)

def main():
    rclpy.init()
    node = BallTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
