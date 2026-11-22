#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_interfaces.msg import BallPos
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        self.current_x = 5.54
        self.current_y = 5.54
        self.current_theta = 0.0
        
        self.balls = {}          
        self.target_id = 1       
        self.is_moving = False   

        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.ball_sub = self.create_subscription(BallPos, '/ball_pos', self.ball_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def ball_callback(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        if len(self.balls) >= 3:
            self.is_moving = True

    def control_loop(self):
        if not self.is_moving:
            return  
            
        if self.target_id > 3:
            self.cmd_pub.publish(Twist()) 
            return

        target_x, target_y = self.balls[self.target_id]

        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        target_theta = math.atan2(dy, dx)
        angle_diff = target_theta - self.current_theta
        

        if distance < 0.5:
            self.get_logger().info(
                f"到达第 {self.target_id} 个球，实际坐标 ({self.current_x:.2f}, {self.current_y:.2f})，误差 {distance:.2f}"
            )
            self.target_id += 1
            return

        cmd = Twist()
        
        if abs(angle_diff) > 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 * angle_diff
        else:
            cmd.linear.x = 1.0 * distance
            cmd.angular.z = 1.5 * angle_diff

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()