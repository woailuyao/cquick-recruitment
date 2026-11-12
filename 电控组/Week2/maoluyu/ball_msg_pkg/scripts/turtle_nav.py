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
        # 发布速度给乌龟
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel",10)
        # 订阅球位置
        self.ball_sub = self.create_subscription(BallPos,"/ball_pos",self.ball_callback,10)
        # 订阅乌龟自身位姿
        self.pose_sub = self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)

        # 存储球数据
        self.ball_dict = {}
        self.target_sequence = [1,2,3] # 必须1→2→3顺序
        self.current_target_idx = 0

        # 乌龟当前状态
        self.turtle_pose = None
        self.reach_threshold = 0.5 # 到达阈值，距离小于0.5判定到达

    def ball_callback(self,msg:BallPos):
        self.ball_dict[msg.id] = {"x":msg.x,"y":msg.y}

    def pose_callback(self,pose:Pose):
        self.turtle_pose = pose
        # 条件：收集齐3个球，乌龟位姿收到，还没走完3个点
        if len(self.ball_dict)>=3 and self.turtle_pose is not None and self.current_target_idx < len(self.target_sequence):
            self.navigate()

    def navigate(self):
        target_id = self.target_sequence[self.current_target_idx]
        target = self.ball_dict[target_id]
        tx = target["x"]
        ty = target["y"]
        cx = self.turtle_pose.x
        cy = self.turtle_pose.y
        ctheta = self.turtle_pose.theta

        # 计算距离和目标角度
        dx = tx - cx
        dy = ty - cy
        dist = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        angle_diff = target_theta - ctheta
        # 角度归一化 [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        twist = Twist()
        if dist < self.reach_threshold:
            # 到达目标点，停止，切换下一个
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"====到达球{target_id}！实际坐标x={cx:.2f},y={cy:.2f},误差dist={dist:.2f}====")
            self.current_target_idx +=1
        else:
            if abs(angle_diff) > 0.15:
                # 角度差距大，原地旋转，线速度0
                twist.linear.x = 0.0
                twist.angular.z = 0.6 * angle_diff
            else:
                # 角度对齐，前进，越靠近速度越小
                twist.angular.z = 0.4*angle_diff
                twist.linear.x = min(0.8, dist * 0.4)
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
