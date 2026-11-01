#!/usr/bin/env python3
# 任务2 · 导航节点（闭环）
# 订阅 ball_pos 收齐 3 个球 → 按 1→2→3 依次走过去
# 订阅 /turtle1/pose 拿乌龟当前位置 → 实时算方位差，先对准再走，越近越慢

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pos.msg import BallPos

# 细则规定的顺序，不许跳、不许倒
ORDER = [1, 2, 3]

TOL = 0.12        # 到点阈值：离目标小于这个距离就算到了
MAX_LIN = 1.6     # 最大线速度 m/s
MAX_ANG = 2.0     # 最大角速度 rad/s
ANG_TOL = 0.25    # 方位角误差小于它，就允许边走边修
KP_DIST = 1.4     # 距离比例系数：越远越快，越近越慢
KP_ANG = 2.2      # 角度比例系数


def norm_angle(a):
    """把角度归一到 -π ~ π，避免绕远路"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class BallNavigator(Node):

    def __init__(self):
        super().__init__('ball_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)

        # 20 Hz 的控制循环：每一拍根据当前状态算一次速度指令
        self.create_timer(0.05, self.control)

        self.pose = None       # 乌龟当前位置 (x, y, theta)
        self.balls = {}        # 按 id 存的球坐标 {1: (x, y), ...}
        self.index = 0         # 当前要去第几个球
        self.finished = False
        self.results = []      # 三个点的最终误差，最后汇总打印

        self.get_logger().info('导航节点已启动，等待收齐 3 个球…')

    def on_ball(self, msg):
        """收到球坐标就往字典里存，按 id 去重"""
        if msg.id not in self.balls:
            self.balls[msg.id] = (msg.x, msg.y)
            self.get_logger().info(
                f'收到球 {msg.id}: ({msg.x:.2f}, {msg.y:.2f})  '
                f'已有 {len(self.balls)}/3')

    def on_pose(self, msg):
        """/turtle1/pose 每来一次就更新一次自己的位置，这就是闭环的反馈"""
        self.pose = msg

    def stop(self):
        self.cmd_pub.publish(Twist())

    def control(self):
        # 三个条件缺一不可：还没完成、知道自己在哪、三个球都收齐了
        if self.finished or self.pose is None or len(self.balls) < 3:
            return

        if self.index >= len(ORDER):
            self.stop()
            self.finished = True
            self.get_logger().info('=== 三个球全部到达 ===')
            for bid, d in self.results:
                self.get_logger().info(f'  球{bid} 误差 {d:.3f}')
            return

        target_id = ORDER[self.index]
        tx, ty = self.balls[target_id]

        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)                       # 到目标的距离
        err = norm_angle(math.atan2(dy, dx) - self.pose.theta)  # 方位差

        cmd = Twist()

        # 到点了：停车、报告、切下一个
        if dist < TOL:
            self.stop()
            self.results.append((target_id, dist))
            self.get_logger().info(
                f'到达球 {target_id}：实际坐标 ({self.pose.x:.2f}, '
                f'{self.pose.y:.2f})，目标 ({tx:.2f}, {ty:.2f})，'
                f'误差 {dist:.3f}')
            self.index += 1
            return

        if abs(err) > ANG_TOL:
            # 方位差太大 → 原地转，先对准，不走
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(KP_ANG * err, -MAX_ANG, MAX_ANG)
        else:
            # 方位对上了 → 边走边微调，距离越近速度越小，防止冲过头
            cmd.linear.x = clamp(KP_DIST * dist, 0.25, MAX_LIN)
            cmd.angular.z = clamp(KP_ANG * err, -MAX_ANG, MAX_ANG)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BallNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
