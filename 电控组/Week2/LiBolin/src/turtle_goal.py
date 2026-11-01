#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ballpos.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleGoalController(Node):
    def __init__(self):
        super().__init__('turtle_goal_controller')
        # 订阅ball_pos获取小球坐标
        self.sub_ball = self.create_subscription(BallPos, 'ball_pos', self.ball_cb, 10)
        # 订阅乌龟自身pose
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        # 发布速度控制
        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # 存储小球：key=id，value=(x,y)
        self.ball_dict = {}
        self.target_seq = [1,2,3] # 强制顺序 1→2→3
        self.current_target_idx = 0
        self.got_all = False
        self.finished = False
        self.error_list = []  # 用来保存三个点的误差

        # 乌龟当前状态
        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.turtle_theta = 0.0

        # 控制参数
        self.dist_thresh = 0.5
        self.angle_thresh = 0.08
        self.kp_ang = 2.0
        self.kp_lin = 0.4

    def ball_cb(self, msg:BallPos):
        if self.got_all:
            return
        self.ball_dict[msg.id] = (msg.x, msg.y)
        if len(self.ball_dict) >=3:
            self.got_all = True

    def pose_cb(self, msg:Pose):
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta

    def wrap_angle(self, ang):
        # 角度归一化 [-pi, pi]
        while ang > math.pi:
            ang -= 2*math.pi
        while ang < -math.pi:
            ang += 2*math.pi
        return ang

    def control_loop(self):
        twist = Twist()
        if not self.got_all:
            # 还没收齐球，不动
            self.pub_cmd.publish(twist)
            return
        if self.current_target_idx >= len(self.target_seq):
            # 全部走完，打印结束提示 + 误差汇总
            if not self.finished:
                self.get_logger().info("3个目标点跑完")
                self.get_logger().info(f"误差汇总：球1={self.error_list[0]:.2f}, 球2={self.error_list[1]:.2f}, 球3={self.error_list[2]:.2f}")
                self.finished = True
            self.pub_cmd.publish(twist)
            return

        # 获取当前目标id
        target_id = self.target_seq[self.current_target_idx]
        tx, ty = self.ball_dict[target_id]
        dx = tx - self.turtle_x
        dy = ty - self.turtle_y
        dist = math.hypot(dx, dy)
        angle_des = math.atan2(dy, dx)
        angle_err = self.wrap_angle(angle_des - self.turtle_theta)

        if dist < self.dist_thresh:
            # 到达该点，保存误差，日志输出，切换下一个
            self.get_logger().info(f"【到达球 {target_id}】实际坐标({self.turtle_x:.2f},{self.turtle_y:.2f})，误差={dist:.2f}")
            self.error_list.append(dist)
            self.current_target_idx +=1
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            if abs(angle_err) > self.angle_thresh:
                # 角度差大 → 原地转，linear.x=0
                twist.angular.z = self.kp_ang * angle_err
                twist.linear.x = 0.0
            else:
                # 对准角度，边走边微调角度
                twist.linear.x = self.kp_lin * dist
                twist.angular.z = self.kp_ang * angle_err
                # 限速，防止冲过头
                if twist.linear.x > 0.8:
                    twist.linear.x = 0.8
        self.pub_cmd.publish(twist)

def main():
    rclpy.init()
    node = TurtleGoalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


