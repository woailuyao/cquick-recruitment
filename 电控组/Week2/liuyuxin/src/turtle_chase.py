#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pos_interfaces.msg import BallPos
import math

class TurtleChaseNode(Node):
    def __init__(self):
        super().__init__('turtle_chase_node')
        # 发布速度控制乌龟
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 订阅小球位置
        self.ball_sub = self.create_subscription(BallPos, '/ball_pos', self.ball_callback, 10)
        # 订阅乌龟自身位姿 x,y,theta
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback,10)

        self.ball_list = []
        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.turtle_theta = 0.0
        self.target_idx = 0
        self.start_move = False
        self.threshold = 0.5 # 到点判定阈值，要求<=0.5

        # 控制定时器
        self.timer = self.create_timer(0.05, self.control_loop)

    def ball_callback(self, msg):
        # 不重复存入相同id
        id_set = [item["id"] for item in self.ball_list]
        if msg.id not in id_set:
            self.ball_list.append({"id":msg.id, "x":msg.x, "y":msg.y})
            self.get_logger().info(f"收到小球 id={msg.id}, x={msg.x:.2f}, y={msg.y:.2f}")
        # 收集满3个球，排序，开始任务
        if len(self.ball_list)>=3 and not self.start_move:
            self.ball_list.sort(key = lambda p: p["id"])
            self.start_move = True
            self.get_logger().info("✅ 3个小球全部接收完毕！按1→2→3依次前往")

    def pose_callback(self,msg):
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta

    def control_loop(self):
        twist = Twist()
        if not self.start_move or self.target_idx >=3:
            self.vel_pub.publish(twist)
            return

        target = self.ball_list[self.target_idx]
        tx = target["x"]
        ty = target["y"]

        dx = tx - self.turtle_x
        dy = ty - self.turtle_y
        dist = math.hypot(dx, dy)
        angle_target = math.atan2(dy, dx)
        angle_err = angle_target - self.turtle_theta
        # 角度误差限制在 -pi ~ pi
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

        # 先对准角度，原地转
        if abs(angle_err) > 0.08:
            twist.angular.z = 0.6 * angle_err
            twist.linear.x = 0.0
        else:
            # 靠近目标时，速度随距离减小，防止冲过头
            twist.linear.x = 0.7 * dist
            twist.angular.z = 0.4 * angle_err
        self.vel_pub.publish(twist)

        # 到达目标，打印规定格式日志
        if dist < self.threshold:
            self.get_logger().info(f"🎉到达第{target['id']}个球，实际坐标({self.turtle_x:.2f},{self.turtle_y:.2f})，误差{dist:.2f}")
            self.target_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = TurtleChaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
