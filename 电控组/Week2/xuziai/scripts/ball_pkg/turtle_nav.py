#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ball_pkg.msg import BallPos
import math

balls_dict = {}       # 存球坐标
current_pose = None   # 存乌龟当前的位置和朝向
target_id = 1         # 当前要去第几个球
is_moving = False     # 是否已经开始动了（收齐3个球才变成 True）

# 订阅球的回调函数
def ball_callback(msg):
    global is_moving
    # 把收到的球存进字典
    balls_dict[msg.id] = (msg.x, msg.y)
    
    # 如果字典里已经有 3 个球了，而且还没开始动，就出发
    if len(balls_dict) == 3 and not is_moving:
        print("收齐3个球，准备出发！")
        is_moving = True

# 获取乌龟位置
def pose_callback(msg):
    global current_pose
    current_pose = msg

# 核心控制逻辑（定时器 10Hz 触发）
def control_callback(node, cmd_pub):
    global target_id, is_moving
    
    # 如果还没收齐球，或者没拿到乌龟坐标，或者三个球都走完了，就啥也不干
    if not is_moving or current_pose is None or target_id > 3:
        return
        
    # 提取目标球的 x, y
    target_x = balls_dict[target_id][0]
    target_y = balls_dict[target_id][1]
    
    # 提取乌龟当前的 x, y, 朝向
    curr_x = current_pose.x
    curr_y = current_pose.y
    curr_theta = current_pose.theta

    # 1. 算距离
    distance = math.sqrt((target_x - curr_x)**2 + (target_y - curr_y)**2)

    # 2. 如果距离小于 0.5，说明到了！
    if distance < 0.5:
        print(f"到达球 {target_id}，实际坐标({curr_x:.2f}, {curr_y:.2f})，误差 {distance:.2f}")
        target_id = target_id + 1  # 切换下一个目标
        
        # 刹车
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        cmd_pub.publish(stop_twist)
        
        if target_id > 3:
            print("所有球都到达了！任务完成！")
        return

    # 3. 算角度（目标相对于乌龟的角度）
    target_angle = math.atan2(target_y - curr_y, target_x - curr_x)
    angle_error = target_angle - curr_theta
    
    # 把角度差限制在 -3.14 到 3.14 之间，防止乌龟绕圈
    if angle_error > math.pi:
        angle_error = angle_error - 2 * math.pi
    elif angle_error < -math.pi:
        angle_error = angle_error + 2 * math.pi

    # 4. 发速度指令
    twist = Twist()
    
    # 如果角度差太大（没对准），原地转
    if abs(angle_error) > 0.2:
        twist.linear.x = 0.0
        twist.angular.z = 1.5 * angle_error
    else:
        # 如果对准了，边走边微调（越近越慢）
        twist.linear.x = 1.0 * distance
        twist.angular.z = 1.5 * angle_error
        
    cmd_pub.publish(twist)

def main(args=None):
    global is_moving
    rclpy.init(args=args)
    node = rclpy.create_node('turtle_navigator')
    
    # 1. 创建发布者（发速度）
    cmd_pub = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    
    # 2. 创建订阅者（收球坐标）
    node.create_subscription(BallPos, 'ball_pos', ball_callback, 10)
    
    # 3. 创建订阅者（收乌龟位置）
    node.create_subscription(Pose, '/turtle1/pose', pose_callback, 10)
    
    # 4. 创建定时器（10Hz，也就是 0.1 秒执行一次 control_callback）
    # 注意：这里用了 lambda 匿名函数，把 node 和 cmd_pub 传给外面的函数
    timer = node.create_timer(0.1, lambda: control_callback(node, cmd_pub))
    
    node.get_logger().info("导航节点已启动，等待接球...")
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()