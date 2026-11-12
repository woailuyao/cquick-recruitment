#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from ball_interfaces.msg import BallPos
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class BallController(Node):
    def __init__(self):
        super().__init__('ball_controller')

        self.pose_received = False
        self.current_target_id = 1
        self.pose = None
        self.done = False
        self.targets = {}
        self.errors = []

        self.ball_sub = self.create_subscription(
            BallPos,'/ball_pos',self.ball_callback,10
        )
        self.pose_sub = self.create_subscription(
            Pose,'/turtle1/pose',self.pose_callback,10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,'/turtle1/cmd_vel',10
        )
        self.timer = self.create_timer(
            0.1,self.control_callback
        )

    def pose_callback(self,msg):
        self.pose = msg
        self.pose_received = True

    def ball_callback(self,msg):
        self.targets[msg.id] = (msg.x,msg.y)

    def control_callback(self):
        if self.done:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        if not self.pose_received:
            return
        if self.current_target_id > 3:
            self.done = True
            cmd = Twist()  # 全零速度
            self.cmd_vel_pub.publish(cmd)
            return
        if self.current_target_id not in self.targets:
            return

        target_x,target_y = self.targets[self.current_target_id]

        turlem_x = self.pose.x
        turlem_y = self.pose.y
        turlem_theta = self.pose.theta

        dx = target_x - turlem_x
        dy = target_y - turlem_y
        target_angle = math.atan2(dy,dx)

        angle_error = target_angle - turlem_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        distance = math.hypot(dx,dy)

        if distance <= 0.5 :
            self.get_logger().info(
            f'到达目标点 {self.current_target_id}: '
            f'目标坐标=({target_x:.2f}, {target_y:.2f}), '
            f'乌龟坐标=({self.pose.x:.2f}, {self.pose.y:.2f}), '                
            f'距离误差={distance:.2f}'
            )

            target_id = self.current_target_id
            self.errors.append(distance)

            if target_id == 3:
                self.done = True
                self.get_logger().info(
                f'三点误差汇总：'
                f'1号={self.errors[0]:.2f}, '
                f'2号={self.errors[1]:.2f}, '
                f'3号={self.errors[2]:.2f}, '
                f'平均={sum(self.errors) / len(self.errors):.3f}, '
                f'最大={max(self.errors):.2f}'
            )
            else:
                self.current_target_id += 1

            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return 

        cmd = Twist()
        if abs(angle_error) > 0.1:
            cmd.angular.z = 1.0 if angle_error > 0 else -1.0
        else:
            cmd.linear.x = 0.2

        self.cmd_vel_pub.publish(cmd)

def main(args = None):
    rclpy.init(args=args)
    node = BallController()
    try:
        rclpy.spin(node)
    except:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



