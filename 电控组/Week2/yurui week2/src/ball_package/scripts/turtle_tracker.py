#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_package.msg import BallPos
class TurtleTracker(Node):
    def __init__(self):
        super().__init__('turtle_tracker')
        # 订阅球坐标 /ball_pos
        self.ball_sub = self.create_subscription(
            BallPos,
            '/ball_pos',
            self.ball_callback,
            10
        )
        # 订阅乌龟当前位置姿态 /turtle1/pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        # 发布速度指令给乌龟 /turtle1/cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 存储目标点，保存已经收到过的球id，防止重复添加
        self.target_list = []
        self.got_id_set = set()
        self.current_target_index = 0  # 当前要去第几个球，从0开始对应球1
        # 控制参数，你选的这套
        self.distance_threshold = 0.1
        self.angle_tolerance = 0.1
        self.linear_speed_max = 1.0
        self.angular_speed_max = 1.0
        self.turtle_pose = None # 保存乌龟实时位姿 x,y,theta
    def ball_callback(self, msg:BallPos):
        """收到球坐标消息，只存新id的球，攒够3个就不再新增"""
        if len(self.target_list) >=3:
            return
        if msg.id not in self.got_id_set:
            self.got_id_set.add(msg.id)
            self.target_list.append({"id":msg.id, "x":msg.x, "y":msg.y})
            self.get_logger().info(f"✅ 收到新目标球 id={msg.id}, x={msg.x}, y={msg.y}")
            self.get_logger().info(f"当前已收集目标数量：{len(self.target_list)}")

            # =====【排序在这里！集齐3个立刻排一次，之后就固定顺序1,2,3】=====
            if len(self.target_list) == 3:
                self.target_list.sort(key=lambda item: item["id"])
                self.get_logger().info("✅ 全部目标接收完毕，已按ID排序，准备开始追踪！")

    def pose_callback(self, msg:Pose):
        """乌龟位置实时回调，闭环核心计算，每次乌龟位置更新就跑这里"""
        if self.turtle_pose is None:
            self.get_logger().info("获取到乌龟初始位姿")
        self.turtle_pose = msg
        # 没有收集齐3个点，不运动
        if len(self.target_list) <3 or self.current_target_index >=3:
            return
        
        # 取出当前目标点
        target = self.target_list[self.current_target_index]
        target_x = target["x"]
        target_y = target["y"]
        # ========== 计算距离误差、角度误差 ==========
        dx = target_x - self.turtle_pose.x
        dy = target_y - self.turtle_pose.y
        distance = math.hypot(dx, dy)
        # 目标点相对于乌龟的方位角
        target_theta = math.atan2(dy, dx)
        angle_error = target_theta - self.turtle_pose.theta
        # 角度归一化，限制在[-pi, pi]，防止角度跳变
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        twist = Twist()
        # 逻辑分支：角度误差大，原地转向；角度对准，前进+微调
        if abs(angle_error) > self.angle_tolerance:
            # 原地旋转，线速度0，只给角速度
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed_max * (1 if angle_error>0 else -1)
        else:
            # 方向对准，前进；距离越近速度越小
            twist.linear.x = self.linear_speed_max * min(1.0, distance * 0.8)
            twist.angular.z = 0.8 * angle_error # 小角度持续微调方向
        # 判断：到达目标点
        if distance < self.distance_threshold:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"到达第 {target['id']} 个球，实际坐标 {self.turtle_pose.x:.2f}, {self.turtle_pose.y:.2f}，误差 {distance:.2f}")
            self.current_target_index += 1
            if self.current_target_index >=3:
                self.get_logger().info("✅✅✅ 全部3个点全部跑完！任务结束！")
            return
        # 发布速度指令给小乌龟
        self.cmd_pub.publish(twist)
def main(args=None):
    rclpy.init(args=args)
    node = TurtleTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
