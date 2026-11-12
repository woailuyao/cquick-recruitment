#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from cquick_ball.msg import BallPos
import math

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.ball_sub = self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.balls = {} # 用来存坐标
        self.targets = []
        self.current_target_index = 0
        self.started = False
        self.finished = False

        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz 控制频率
        self.get_logger().info("导航节点已启动，等待收齐 3 个球...")

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def ball_callback(self, msg):
        if self.started: return
        self.balls[msg.id] = (msg.x, msg.y)
        # 收齐 3 个再动
        if len(self.balls) == 3 and not self.started:
            self.started = True
            # 按 id 排序 1 -> 2 -> 3
            self.targets = [self.balls[i] for i in sorted(self.balls.keys())]
            self.get_logger().info("已收齐3个球坐标，开始导航！")

    def control_loop(self):
        if not self.started or self.finished:
            return

        if self.current_target_index >= len(self.targets):
            self.finished = True
            self.stop_robot()
            self.get_logger().info("所有目标已到达，任务完成！")
            return

        target_x, target_y = self.targets[self.current_target_index]
        target_id = self.current_target_index + 1

        # 计算距离和角度
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        # 到点判定：误差 < 0.5
        if distance < 0.5:
            self.get_logger().info(f'到达球 {target_id}，实际坐标 ({self.current_x:.2f}, {self.current_y:.2f})，误差 {distance:.2f}')
            self.current_target_index += 1
            self.stop_robot()
            return

        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_theta

        # 角度归一化
        while angle_diff > math.pi: angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi: angle_diff += 2.0 * math.pi

        twist = Twist()

        # 先对准再走
        if abs(angle_diff) > 0.2:
            twist.linear.x = 0.0
            twist.angular.z = 1.5 * angle_diff 
        else:
            # 边走边微调，越近越慢
            twist.linear.x = min(1.5 * distance, 1.5)
            twist.angular.z = 1.5 * angle_diff

        self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()