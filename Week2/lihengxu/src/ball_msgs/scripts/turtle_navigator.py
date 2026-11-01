#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_msgs.msg import BallPos

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')
        
        # 初始化目标点 (根据你任务1的设置：球1, 球2, 球3)
        # 注意：turtlesim 的坐标系左上角是(0,0)，往右x增加，往下y增加
        self.waypoints = [
            {'id': 1, 'x': 2.0, 'y': 3.0},
            {'id': 2, 'x': 5.5, 'y': 8.0},
            {'id': 3, 'x': 9.0, 'y': 4.0}
        ]
        self.current_waypoint_index = 0
        self.collected_points = []
        
        # 订阅者：订阅话题获取目标坐标
        self.subscription = self.create_subscription(
            BallPos,
            '/ball_pos',
            self.ball_pos_callback,
            10
        )
        
        # 订阅者：订阅乌龟当前姿态
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        # 发布者：发布速度指令控制乌龟
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 初始化数据
        self.turtle_pose = Pose()
        self.target_point = None
        self.reached = False
        self.timer = self.create_timer(0.1, self.navigate_callback) # 10Hz的控制频率
        
        self.get_logger().info('Turtle Navigator Started. Waiting for ball positions...')

    def ball_pos_callback(self, msg):
        # 接收球的位置，并按 id 顺序收集
        if not self.collected_points or msg.id not in [p['id'] for p in self.collected_points]:
            self.collected_points.append({'id': msg.id, 'x': msg.x, 'y': msg.y})
            self.get_logger().info(f'收到球 {msg.id} 的坐标: ({msg.x}, {msg.y})')
            
            # 收到三个球后，开始导航
            if len(self.collected_points) >= 3:
                if self.target_point is None:
                    self.set_next_target()

    def pose_callback(self, msg):
        # 记录乌龟当前状态
        self.turtle_pose = msg

    def set_next_target(self):
        if self.current_waypoint_index < len(self.waypoints):
            self.target_point = self.collected_points[self.current_waypoint_index]
            self.get_logger().info(f'设置下一个目标: 球{self.target_point["id"]}, 坐标 ({self.target_point["x"]}, {self.target_point["y"]})')
            self.reached = False

    def navigate_callback(self):
        # 如果还没收齐球，或者已经走完了所有点，就不发送速度
        if not self.collected_points or len(self.collected_points) < 3:
            return
        if self.current_waypoint_index >= len(self.collected_points):
            return

        if self.target_point is None:
            return

        # 计算当前点与目标点的距离和角度差
        dx = self.target_point['x'] - self.turtle_pose.x
        dy = self.target_point['y'] - self.turtle_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # 计算目标点的方位角 (theta_goal)
        theta_goal = math.atan2(dy, dx)
        
        # 计算角度误差
        angle_error = theta_goal - self.turtle_pose.theta
        
        # 角度归一化到 -pi 到 pi 之间，防止跳变
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi
            
        twist = Twist()
        
        # 核心控制逻辑：先对准，再前进
        if abs(angle_error) > 0.1: # 角度误差较大，原地旋转对准
            twist.angular.z = 1.5 * angle_error # P控制，比例系数
            twist.linear.x = 0.0
        else: # 角度对准了，开始向前走
            twist.linear.x = 1.0 * distance # P控制，距离越远速度越快
            twist.angular.z = 0.5 * angle_error # 微调方向
            
        self.publisher_.publish(twist)

        # 判断是否到达目标 (误差 <= 0.5)
        if distance < 0.5 and not self.reached:
            self.reached = True
            self.get_logger().info(f'到达球 {self.target_point["id"]}! 实际坐标 ({self.turtle_pose.x:.2f}, {self.turtle_pose.y:.2f}), 误差 {distance:.2f}')
            
            # 切换到下一个点
            self.current_waypoint_index += 1
            self.set_next_target()
            
            # 走完所有点后的收尾
            if self.current_waypoint_index >= len(self.collected_points):
                self.get_logger().info('所有目标点已到达，任务完成！停止运动。')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                # 停止定时器防止重复执行
                self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
