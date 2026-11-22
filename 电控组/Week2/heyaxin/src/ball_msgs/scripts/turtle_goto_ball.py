#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
# =========这里包名是 ball_msgs，不要写错！=========
from ball_msgs.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math
import datetime


class TurtleGotoBall(Node):
    def __init__(self):
        super().__init__("turtle_goto_ball_node")

        # 发布速度指令给乌龟 /turtle1/cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # 订阅球坐标话题 /ball_pos（任务1的话题名）
        self.ball_sub = self.create_subscription(
            BallPos,
            "/ball_pos",
            self.ball_callback,
            10
        )

        # 订阅乌龟自身位姿 /turtle1/pose，闭环反馈
        self.pose_sub = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10
        )

        # 定时器10Hz，0.1s执行一次控制逻辑；❌不要在回调写while循环
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_control_loop)

        # 存储球坐标字典 key=球id value=(x,y)
        self.ball_dict = {}
        # 严格顺序：1→2→3，考核硬性要求
        self.target_id_list = [1, 2, 3]
        self.current_target_idx = 0

        # 乌龟实时状态
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_theta = 0.0

        # 控制参数（文档要求距离阈值≤0.5）
        self.dist_threshold = 0.5
        self.angle_threshold = 0.15
        self.max_linear_speed = 0.6
        self.max_angular_speed = 0.8

        self.get_logger().info(f"[heyaxin {self.stamp()}] 任务2节点启动，等待收集全部3个球坐标...")

    def stamp(self):
        # 每条日志都带「姓名 + 实时时间戳」，截图天然满足防抄袭要求
        return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    # 球消息回调：只存数据，不做运动！
    def ball_callback(self, msg: BallPos):
        self.ball_dict[msg.id] = (msg.x, msg.y)

    # 乌龟位姿回调，保存实时位置角度
    def pose_callback(self, msg: Pose):
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_theta = msg.theta

    # 角度归一化：把角度约束 [-pi , pi]，解决角度跳变大转弯bug
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # 核心控制循环，定时器每0.1s运行一次
    def timer_control_loop(self):
        twist = Twist()

        # 没有收齐3个球，直接返回，不动作
        if len(self.ball_dict) < 3:
            return

        # 获取当前目标id与坐标
        target_id = self.target_id_list[self.current_target_idx]
        target_x, target_y = self.ball_dict[target_id]

        dx = target_x - self.curr_x
        dy = target_y - self.curr_y
        dist = math.hypot(dx, dy)

        desired_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(desired_theta - self.curr_theta)

        # 判断到达目标点
        if dist < self.dist_threshold:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            # 考核必须打印日志：到达id、实际坐标、误差
            self.get_logger().info(
                f"[heyaxin {self.stamp()}] ✅到达球{target_id},实际坐标({self.curr_x:.2f},{self.curr_y:.2f}),误差dist={dist:.2f}"
            )
            # 切换下一个目标
            self.current_target_idx += 1
            # 判断全部跑完
            if self.current_target_idx >= len(self.target_id_list):
                self.get_logger().info(
                    f"[heyaxin {self.stamp()}] 🎉三个球全部访问完毕！任务2结束！"
                )
                self.destroy_timer(self.timer)
            return

        # 未到达，运动逻辑
        if abs(angle_error) > self.angle_threshold:
            # 角度差大，原地旋转，不前进
            twist.linear.x = 0.0
            if angle_error > 0:
                twist.angular.z = self.max_angular_speed
            else:
                twist.angular.z = -self.max_angular_speed
        else:
            # 角度对准，前进；距离越近速度越小，防止冲过头
            twist.linear.x = min(self.max_linear_speed, dist * 0.6)
            twist.angular.z = 0.3 * angle_error

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleGotoBall()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()