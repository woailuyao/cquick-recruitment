#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pos_pkg.msg import BallPos
import math


class TurtleGoalsNode(Node):
    def __init__(self):
        super().__init__("turtle_goals")

        # 存储球坐标
        self.balls_dict = {}
        # 按顺序排列的目标点列表
        self.goal_list = []
        self.current_goal_idx = 0

        # 乌龟当前位姿
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_theta = 0.0
        self.pose_ready = False

        # 控制参数
        self.dist_threshold = 0.3   # 到达判定阈值
        self.angle_threshold = 0.05 # 角度对准阈值，单位rad
        self.kp_linear = 1.2        # 线速度比例系数
        self.kp_angular = 2.5       # 角速度比例系数
        self.max_linear = 2.0       # 最大线速度限制
        self.max_angular = 2.5      # 最大角速度限制

        # 订阅者：球坐标话题
        self.ball_sub = self.create_subscription(
            BallPos, "ball_pos", self.ball_callback, 10
        )
        # 订阅者：乌龟位姿
        self.pose_sub = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        # 发布者：速度控制指令
        self.cmd_pub = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )
        # 10Hz控制定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("追球节点启动，等待接收球坐标...")

    def ball_callback(self, msg: BallPos):
        """接收球坐标，收齐3个后按1→2→3排序生成目标列表"""
        self.balls_dict[msg.id] = (msg.x, msg.y)
        # 收齐3个且还没初始化目标列表
        if len(self.balls_dict) == 3 and len(self.goal_list) == 0:
            for ball_id in [1, 2, 3]:
                self.goal_list.append(self.balls_dict[ball_id])
            self.get_logger().info(f"已收齐3个球，目标顺序：{self.goal_list}")

    def pose_callback(self, msg: Pose):
        """更新乌龟实时位姿"""
        self.cur_x = msg.x
        self.cur_y = msg.y
        self.cur_theta = msg.theta
        self.pose_ready = True

    def normalize_angle(self, angle):
        """将角度归一化到 [-π, π]，避免反向转圈"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        """主控制循环：状态判断 + 速度计算"""
        # 未就位：没收到位姿 / 没收齐球，不输出速度
        if not self.pose_ready or len(self.goal_list) == 0:
            return

        # 全部目标走完，停止并结束
        if self.current_goal_idx >= len(self.goal_list):
            self.cmd_pub.publish(Twist())
            if self.current_goal_idx == len(self.goal_list):
                self.get_logger().info("=== 所有目标点到达完成，任务结束 ===")
                self.current_goal_idx += 1
            return

        # 获取当前目标点
        goal_x, goal_y = self.goal_list[self.current_goal_idx]
        # 计算距离与目标角度
        dx = goal_x - self.cur_x
        dy = goal_y - self.cur_y
        distance = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        angle_err = self.normalize_angle(target_theta - self.cur_theta)

        twist = Twist()

        # 角度偏差大：原地转向对准
        if abs(angle_err) > self.angle_threshold:
            twist.angular.z = self.kp_angular * angle_err
            # 角速度限幅
            twist.angular.z = max(-self.max_angular, min(self.max_angular, twist.angular.z))
        else:
            # 对准后前进，同时微调方向
            twist.linear.x = self.kp_linear * distance
            twist.angular.z = self.kp_angular * angle_err
            # 速度限幅
            twist.linear.x = min(self.max_linear, twist.linear.x)
            twist.angular.z = max(-self.max_angular, min(self.max_angular, twist.angular.z))

        # 到达目标判定
        if distance < self.dist_threshold:
            twist = Twist()  # 立即停车
            self.cmd_pub.publish(twist)
            self.get_logger().info(
                f"到达球 {self.current_goal_idx + 1} | "
                f"实际坐标 ({self.cur_x:.2f}, {self.cur_y:.2f}) | "
                f"误差 {distance:.2f}"
            )
            self.current_goal_idx += 1
            return

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleGoalsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 时强制停止乌龟
        node.cmd_pub.publish(Twist())
        node.get_logger().info("手动中断，乌龟已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
