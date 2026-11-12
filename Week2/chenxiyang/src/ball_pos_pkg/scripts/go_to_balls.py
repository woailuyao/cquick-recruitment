#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pos_pkg.msg import BallPos
import math

class GoToBalls(Node):
    def __init__(self):
        super().__init__('go_to_balls')
        # 1. 发布速度指令
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 2. 订阅乌龟当前位置
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # 3. 订阅球的位置
        self.ball_sub = self.create_subscription(BallPos, 'ball_pos', self.ball_callback, 10)
        
        self.current_pose = None
        self.balls = {}  # 字典存球，按 id 区分
        self.target_id = 1  # 当前要去哪个球
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz 控制循环

        self.get_logger().info('导航节点已启动，等待收齐 3 个球...')

    def pose_callback(self, msg):
        self.current_pose = msg

    def ball_callback(self, msg):
        # 把收到的球存起来，用 id 当 key
        self.balls[msg.id] = (msg.x, msg.y)
        # 如果收齐了 3 个，且还没开始打印过，提示一下
        if len(self.balls) == 3 and self.target_id == 1:
            self.get_logger().info('收齐 3 个球，开始按 1->2->3 依次导航！')

    def control_loop(self):
        # 没拿到位置，或者球没收齐，或者已经跑完 3 个球，就不动
        if self.current_pose is None or len(self.balls) < 3 or self.target_id > 3:
            return

        # 1. 拿到目标坐标
        target_x, target_y = self.balls[self.target_id]
        curr_x = self.current_pose.x
        curr_y = self.current_pose.y
        curr_theta = self.current_pose.theta

        # 2. 算距离和角度差
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.sqrt(dx**2 + dy**2)
        
        target_theta = math.atan2(dy, dx)
        # 把角度差限制在 -pi 到 pi 之间（处理绕圈）
        angle_diff = math.atan2(math.sin(target_theta - curr_theta), math.cos(target_theta - curr_theta))

        twist = Twist()

        # 3. 到点判断：距离小于 0.5 就算到了
        if distance < 0.5:
            self.get_logger().info(f'到达球 {self.target_id}！实际坐标: ({curr_x:.2f}, {curr_y:.2f}), 误差: {distance:.2f}')
            self.target_id += 1
            # 停下
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            if self.target_id > 3:
                self.get_logger().info('所有 3 个球都走完啦！')
            return

        # 4. 闭环控制：角度差大就先原地转，角度对准了就走
        if abs(angle_diff) > 0.1:
            # 原地转（只给角速度）
            twist.linear.x = 0.0
            twist.angular.z = 1.0 * angle_diff  # 比例控制
        else:
            # 边走边微调（越近越慢：速度跟距离挂钩，最大 2.0，最小 0.2）
            twist.linear.x = min(2.0, 1.5 * distance + 0.2)
            twist.angular.z = 1.0 * angle_diff

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = GoToBalls()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
