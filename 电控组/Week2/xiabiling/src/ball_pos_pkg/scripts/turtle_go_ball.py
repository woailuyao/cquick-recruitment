#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleNavNode(Node):
    def __init__(self):
        super().__init__("turtle_nav_node")

        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.ball_sub = self.create_subscription(
            BallPos, "ball_pos", self.ball_callback, 10
        )
        self.pose_sub = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )

        self.ball_target = {}
        self.order = [1,2,3]
        self.current_target_idx = 0
        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.turtle_theta = 0.0

        self.reach_threshold = 0.5
        self.max_linear = 1.0
        self.max_angular = 2.0

        self.get_logger().info("导航节点启动，等待接收3个球坐标...")

    def ball_callback(self, msg:BallPos):
        """收到BallPos消息，存到字典"""
        self.ball_target[msg.id] = (float(msg.x), float(msg.y))

    def pose_callback(self, msg:Pose):
        """乌龟实时位姿回调，核心控制逻辑在这里"""
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta

        if len(self.ball_target) <3:
            return
        if self.current_target_idx >= len(self.order):
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)
            self.get_logger().info("✅全部目标点走完！")
            return

        target_id = self.order[self.current_target_idx]
        t_x, t_y = self.ball_target[target_id]

        dx = t_x - self.turtle_x
        dy = t_y - self.turtle_y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)

        angle_err = target_angle - self.turtle_theta
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

        twist = Twist()
        if dist < self.reach_threshold:
            twist.linear.x = 0.0
            twist.angular.z =0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info(
                f"🎯到达球{target_id},实际坐标({self.turtle_x:.2f},{self.turtle_y:.2f}),误差={dist:.2f}"
            )
            self.current_target_idx +=1
            return

        if abs(angle_err) > 0.15:
            twist.angular.z = self.max_angular * (angle_err / abs(angle_err))
            twist.linear.x =0.0
        else:
            twist.angular.z = 0.6 * angle_err
            twist.linear.x = min(self.max_linear, dist*0.8)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()

