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
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.ball_sub = self.create_subscription(BallPos, '/ball_pos', self.ball_cb, 10)
        self.balls = {}
        self.pose = None
        self.target_id = 1
        self.all_arrived = False
        
        self.k_linear = 1.0
        self.k_angular = 4.0
        self.max_linear = 2.0
        self.max_angular = 2.0
        self.angle_tol = 0.1
        self.dist_tol = 0.15
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_cb(self, msg):
        self.pose = msg

    def ball_cb(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        if len(self.balls) >= 3 and self.target_id == 1 and not self.all_arrived:
            self.get_logger().info(f'收齐 3 个球: {self.balls}，开始依次前往')

    def control_loop(self):
        if self.pose is None or len(self.balls) < 3 or self.all_arrived:
            return
        if self.target_id not in self.balls:
            return
        tx, ty = self.balls[self.target_id]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        angle_err = self._normalize(target_theta - self.pose.theta)
        cmd = Twist()

        if dist < self.dist_tol:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'到达球 {self.target_id} | 目标 ({tx:.2f},{ty:.2f}) | 实际 ({self.pose.x:.3f},{self.pose.y:.3f}) | 误差 {dist:.3f}')
            self.target_id += 1
            if self.target_id > 3:
                self.all_arrived = True
                self.get_logger().info('三个球全部到达，任务完成')
            return

        if abs(angle_err) > self.angle_tol:
            cmd.linear.x = 0.0
            cmd.angular.z = self._clamp(self.k_angular * angle_err, self.max_angular)
        else:
            cmd.linear.x = min(self.max_linear, self.k_linear * dist)
            cmd.angular.z = self._clamp(self.k_angular * angle_err, self.max_angular)
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _normalize(a):
        while a > math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    @staticmethod
    def _clamp(v, lim):
        return max(-lim, min(lim, v))

def main(args=None):
    rclpy.init(args=args)
    node = BallNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
