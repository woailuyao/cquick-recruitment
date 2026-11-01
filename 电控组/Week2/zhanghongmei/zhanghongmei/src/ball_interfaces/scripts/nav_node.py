#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_interfaces.msg import BallPos
import math

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.sub_ball = self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        self.current_pose = None
        self.balls = {}
        self.targets = []
        self.current_target = None

    def pose_cb(self, msg):
        self.current_pose = msg

    def ball_cb(self, msg):
        if msg.id not in self.balls:
            self.balls[msg.id] = (msg.x, msg.y)
            self.get_logger().info(f'收到球 {msg.id}: ({msg.x}, {msg.y})')
            if len(self.balls) == 3:
                self.targets = [self.balls[1], self.balls[2], self.balls[3]]
                self.get_logger().info('3个球已收齐，开始按 1->2->3 顺序导航！')

    def run(self):
        rate = self.create_rate(10)
        while rclpy.ok():
            if self.current_target is None and self.targets:
                self.current_target = self.targets.pop(0)
                self.get_logger().info(f'前往目标: {self.current_target}')
            elif self.current_target is not None and self.current_pose:
                goal_x, goal_y = self.current_target
                x, y, theta = self.current_pose.x, self.current_pose.y, self.current_pose.theta
                
                dist = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
                angle_to_goal = math.atan2(goal_y - y, goal_x - x)
                angle_diff = math.atan2(math.sin(angle_to_goal - theta), math.cos(angle_to_goal - theta))
                
                cmd = Twist()
                if dist > 0.1: 
                    if abs(angle_diff) > 0.3:
                        cmd.linear.x = 0.0
                        cmd.angular.z = 1.5 if angle_diff > 0 else -1.5
                    else:
                        cmd.linear.x = min(1.5 * dist, 2.0)
                        cmd.angular.z = 1.5 * angle_diff
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info(f'=== 到达目标！实际坐标: ({x:.2f}, {y:.2f}), 误差: {dist:.2f} ===')
                    self.current_target = None
                
                self.pub.publish(cmd)
            rate.sleep()

def main():
    rclpy.init()
    node = NavNode()
    import threading
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
