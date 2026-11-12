#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_msg_pkg.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleNavNode(Node):
    def __init__(self):
        super().__init__("turtle_nav_node")
        # 发布twist控制乌龟
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # 订阅球位置话题
        self.ball_sub = self.create_subscription(BallPos, "/ball_pos", self.ball_callback,10)
        # 订阅乌龟自身位置pose
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback,10)

        # 存储球坐标字典
        self.ball_dict = {}
        # 目标点顺序：1,2,3
        self.target_seq = [1,2,3]
        self.current_target_idx = 0
        self.got_all_ball = False

        # 乌龟当前状态
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_theta = 0.0

        # 控制参数
        self.dist_threshold = 0.5    # 到达判定阈值，任务要求≤0.5
        self.angle_kp = 1.2         # 角度比例系数
        self.linear_kp = 0.6        # 线速度比例系数

    # 接收球坐标回调
    def ball_callback(self,msg:BallPos):
        if msg.id not in self.ball_dict:
            self.ball_dict[msg.id] = (msg.x, msg.y)
            self.get_logger().info(f"收到球id={msg.id}，坐标x={msg.x},y={msg.y}")
        # 判断是否集齐3个球
        if len(self.ball_dict) ==3 and not self.got_all_ball:
            self.got_all_ball = True
            self.get_logger().info("✅ 全部3个球坐标收集完成！开始导航：1→2→3")

    # 获取乌龟实时位置
    def pose_callback(self,pose:Pose):
        self.cur_x = pose.x
        self.cur_y = pose.y
        self.cur_theta = pose.theta
        if not self.got_all_ball:
            return
        # 取出当前目标id
        target_id = self.target_seq[self.current_target_idx]
        t_x, t_y = self.ball_dict[target_id]

        # 计算距离误差、目标方位角
        dx = t_x - self.cur_x
        dy = t_y - self.cur_y
        dist_err = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_err = target_angle - self.cur_theta
        # 角度归一化到 [-pi,pi]
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

        twist = Twist()
        # 判定到达目标
        if dist_err < self.dist_threshold:
            twist.linear.x =0.0
            twist.angular.z =0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"🎉 到达球{target_id}！实际坐标({self.cur_x:.2f},{self.cur_y:.2f}),误差={dist_err:.2f}")
            self.current_target_idx +=1
            if self.current_target_idx >= len(self.target_seq):
                self.get_logger().info("✅ 全部3个点位跑完，导航结束！")
                self.destroy_node()
            return
        # 角度差大：原地旋转，不前进
        if abs(angle_err) > 0.15:
            twist.angular.z = self.angle_kp * angle_err
            twist.linear.x =0.0
        # 角度对准，向前走，边前进微调角度
        else:
            twist.linear.x = self.linear_kp * dist_err
            twist.angular.z = self.angle_kp * angle_err
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

