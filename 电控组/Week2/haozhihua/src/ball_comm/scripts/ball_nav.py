#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_comm.msg import BallPos

# ===== 可调参数：调这几个数字就是"整定" =====
MAX_LIN  = 1.2     # 最大线速度 m/s
MAX_ANG  = 2.0     # 最大角速度 rad/s
K_LIN    = 1.0     # 线速度比例系数
K_ANG    = 2.5     # 角速度比例系数
DIST_TOL = 0.15    # 到点判定阈值（细则要求最终误差 ≤ 0.5，留足余量）
ANG_TOL  = 0.06    # "对准了"的角度阈值 rad（约 3.4°）
ORDER    = [1, 2, 3]   # 必须按这个顺序


class BallNav(Node):
    def __init__(self):
        super().__init__('ball_nav')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)

        self.pose  = None    # 乌龟当前位姿
        self.balls = {}      # id -> (x, y)
        self.idx   = 0       # 正在去第几个球
        self.done  = False

        self.timer = self.create_timer(0.05, self.control)   # 20 Hz

    def on_pose(self, msg):
        self.pose = msg

    def on_ball(self, msg):
        if msg.id not in self.balls:
            self.get_logger().info(f'[规划] 记录球 {msg.id} -> ({msg.x}, {msg.y})')
        self.balls[msg.id] = (msg.x, msg.y)

    def control(self):
        # 还没拿到自己的位姿 或 已经跑完 -> 什么都不做
        if self.done or self.pose is None:
            return

        # TODO 1：还没收齐 3 个球就原地等（提示：判断 len(self.balls)）
        if len(self.balls) < len(ORDER):
            return

        # 三个都到了 -> 停车收工
        if self.idx >= len(ORDER):
            self.stop()
            self.done = True
            self.get_logger().info('三个球全部到达，任务结束')
            return

        bid = ORDER[self.idx]
        tx, ty = self.balls[bid]

        # ---- 第 1 句：算距离和目标方向 ----
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist   = math.hypot(dx, dy)
        target = math.atan2(dy, dx)

        # ---- 第 2 句：算角度差并折到 [-π, π] ----
        err = math.atan2(math.sin(target - self.pose.theta),
                         math.cos(target - self.pose.theta))

        # ---- 第 3 句：到点判定 ----
        if dist <= DIST_TOL:
            self.stop()
            self.get_logger().info(
                f'到达球 {bid}，实际坐标 ({self.pose.x:.2f}, {self.pose.y:.2f})，'
                f'目标 ({tx}, {ty})，误差 {dist:.3f}')
            self.idx += 1
            return

        # ---- 第 3 句：决定速度 ----
        cmd = Twist()
        if abs(err) > ANG_TOL:
            # TODO 2：还没对准 -> 原地转（linear.x 给多少？angular.z 怎么算？记得限幅）
            cmd.linear.x  = 0.0
            cmd.angular.z = max(-MAX_ANG, min(MAX_ANG , K_ANG * err))
        else:
            # TODO 3：对准了 -> 边走边微调，越近越慢
            cmd.linear.x  = max(-MAX_LIN, min(MAX_LIN , K_LIN * dist))
            cmd.angular.z = 0.0

        self.pub.publish(cmd)

    def stop(self):
        self.pub.publish(Twist())      # 空的 Twist 就是全 0 = 停车


def main():
    rclpy.init()
    node = BallNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()