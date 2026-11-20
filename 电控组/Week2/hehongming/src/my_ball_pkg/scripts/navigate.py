#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_ball_pkg.msg import BallPos
import math

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')
        # 1. 创建速度发布者
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 2. 创建位置订阅者 (获取乌龟当前坐标)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # 3. 创建球坐标订阅者
        self.ball_sub = self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)
        
        # 状态变量
        self.balls = {}           # 存球的字典 {id: (x, y)}
        self.current_pose = None  # 乌龟当前位置
        self.current_target_id = 1 # 当前要去追的球 ID
        
        # 控制频率 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg

    def ball_callback(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)

    def control_loop(self):
        # 如果还没收到乌龟位置，或者没收齐3个球，或者3个球都追完了，就停车并返回
        if self.current_pose is None or len(self.balls) < 3 or self.current_target_id > 3:
            self.stop_robot()
            if self.current_target_id > 3:
                self.get_logger().info('所有球已追完！', once=True)
            return

        target_x, target_y = self.balls[self.current_target_id]
        curr_x = self.current_pose.x
        curr_y = self.current_pose.y
        curr_theta = self.current_pose.theta

        # 计算目标点到当前点的距离和角度差
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.sqrt(dx**2 + dy**2)

        # 1. 到达判定 (误差小于0.2算到达，留点余量)
        if distance < 0.2:
            self.get_logger().info(f'到达球 {self.current_target_id}！实际坐标({curr_x:.2f}, {curr_y:.2f})，误差 {distance:.2f}')
            self.current_target_id += 1
            return

        # 2. 计算目标角度 (-pi 到 pi)
        target_theta = math.atan2(dy, dx)
        # 角度差 (归一化到 -pi 到 pi 之间)
        angle_diff = target_theta - curr_theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        twist = Twist()
        # 3. 先对准再走策略
        if abs(angle_diff) > 0.2:
            # 角度偏差大，原地转
            twist.linear.x = 0.0
            twist.angular.z = 1.5 * angle_diff
        else:
            # 已经大致对准，边走边微调，越近越慢
            twist.linear.x = 1.0 * distance
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