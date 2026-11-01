#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_interfaces.msg import BallPos
import math

class TurtleNav(Node):
    def __init__(self):
        super().__init__('turtle_nav_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.pose_sub = self.create_subscription(Pose,'/turtle1/pose',self.pose_cb,10)
        self.ball_sub = self.create_subscription(BallPos,'ball_pos',self.ball_cb,10)

        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_theta = 0.0

        self.ball_dict = {}
        self.target_seq = [1,2,3]
        self.current_target_idx = 0

        self.distance_tol = 0.45
        self.angle_tol = 0.12
        self.k_ang = 2.2
        self.k_lin = 0.7

        self.got_all_ball = False
        self.control_timer = self.create_timer(0.1,self.control_loop)
        self.get_logger().info("导航节点启动，等待接收全部3个球坐标...")

    def pose_cb(self,msg):
        self.cur_x = msg.x
        self.cur_y = msg.y
        self.cur_theta = msg.theta

    def ball_cb(self,msg):
        self.ball_dict[msg.id] = (float(msg.x),float(msg.y))
        if len(self.ball_dict)>=3 and 1 in self.ball_dict and 2 in self.ball_dict and 3 in self.ball_dict:
            if not self.got_all_ball:
                self.got_all_ball = True
                self.get_logger().info("✅已经集齐全部3个球坐标，开始导航！")

    @staticmethod
    def normalize_angle(angle):
        while angle>math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def control_loop(self):
        twist = Twist()
        if not self.got_all_ball:
            self.cmd_vel_pub.publish(twist)
            return
        target_id = self.target_seq[self.current_target_idx]
        t_x,t_y = self.ball_dict[target_id]
        dx = t_x - self.cur_x
        dy = t_y - self.cur_y
        dist = math.hypot(dx,dy)
        desired_angle = math.atan2(dy,dx)
        angle_err = self.normalize_angle(desired_angle - self.cur_theta)

        if dist < self.distance_tol:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            err = dist
            self.get_logger().info(f"🎉到达球{target_id},实际坐标({self.cur_x:.2f},{self.cur_y:.2f}),误差={err:.2f}")
            self.current_target_idx +=1
            if self.current_target_idx >= len(self.target_seq):
                self.get_logger().info("🏁全部3个目标点跑完，停止运动！")
                self.destroy_timer(self.control_timer)
            return

        if abs(angle_err) > self.angle_tol:
            twist.linear.x = 0.0
            twist.angular.z = self.k_ang * angle_err
        else:
            twist.angular.z = self.k_ang * angle_err
            twist.linear.x = self.k_lin * dist
            if twist.linear.x>1.0:
                twist.linear.x =1.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
