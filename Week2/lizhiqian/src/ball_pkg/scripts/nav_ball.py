#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class BallNavNode(Node):
    def __init__(self):
        super().__init__("nav_ball_node")
        # 订阅球坐标
        self.ball_sub = self.create_subscription(BallPos, "ball_pos", self.ball_cb, 10)
        # 订阅乌龟真实位姿！这是算误差的关键
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_cb, 10)
        # 速度发布
        self.vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.ball_dict = {}
        self.ready = False
        self.target_seq = [1,2,3]
        self.target_idx = 0

        # 乌龟真实坐标，从pose回调刷新
        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.turtle_theta = 0.0

        self.timer = self.create_timer(0.1, self.nav_timer)

    def ball_cb(self, msg):
        self.ball_dict[msg.id] = (msg.x, msg.y)
        self.get_logger().info(f"收到球 id={msg.id}, x={msg.x}, y={msg.y}")
        if len(self.ball_dict) == 3 and not self.ready:
            self.ready = True
            self.get_logger().info("三个球坐标全部收齐，开始导航 1→2→3")

    def pose_cb(self, msg):
        # 实时拿到乌龟真实位置
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta

    def nav_timer(self):
        twist = Twist()
        if not self.ready or self.target_idx >= len(self.target_seq):
            self.vel_pub.publish(twist)
            return

        tid = self.target_seq[self.target_idx]
        tx, ty = self.ball_dict[tid]
        dx = tx - self.turtle_x
        dy = ty - self.turtle_y
        dist = math.hypot(dx, dy)

        # 到达判定阈值0.5
        if dist < 0.5:
            self.get_logger().info(f"到达第{tid}个球，实际坐标({self.turtle_x:.2f},{self.turtle_y:.2f})，误差 d={dist:.3f}")
            self.target_idx += 1
            if self.target_idx >= len(self.target_seq):
                self.get_logger().info("全部点跑完，停止！")
            return

        # 角度控制：先对准目标方向
        angle_target = math.atan2(dy, dx)
        angle_err = angle_target - self.turtle_theta
        # 角度限幅 -pi~pi
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

        # 控制逻辑：角度误差大，优先转角度；对准了再前进
        if abs(angle_err) > 0.12:
            twist.angular.z = 1.8 * angle_err
            twist.linear.x = 0.0
        else:
            twist.linear.x = 1.2 * dist
            twist.angular.z = 0.6 * angle_err

        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BallNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()