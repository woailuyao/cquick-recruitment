#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist         #v
from turtlesim.msg import Pose
from ball_interfaces.msg import BallPos

def normalize_angle(a):
    #[-pi,+pi]
    while a > math.pi:
        a -= 2 * math.pi
    while a < - math.pi:
        a += 2 * math.pi
    return a

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')

        #(x,y)
        self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)
        self.balls = {}
        self.target_order = [1, 2, 3]
        self.current = 0

        #pose
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None

        #v
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.create_timer(0.1, self.control_loop)

    def ball_callback(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None or len(self.balls) < 3:
            return
        if self.current >= len(self.target_order):
            return

        twist = Twist()
        tx, ty = self.balls[self.target_order[self.current]]

        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_err = normalize_angle(angle_to_target - self.pose.theta)


        if abs(angle_err) > 0.15:
            twist.angular.z = 1.5 * angle_err

        else:
            twist.linear.x = min(2.0 * dist, 1.5)
            twist.angular.z = 0.8 * angle_err


        if dist < 0.1:
            self.get_logger().info(
                f'到达球{self.target_order[self.current]}！'
                f'实际坐标 ({self.pose.x:.2f}, {self.pose.y:.2f})，'
                f'误差 {dist:.3f}')
            self.current += 1
            if self.current >= len(self.target_order):
                self.get_logger().info('三个球全部到达，任务完成！')
                return

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = TurtleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()