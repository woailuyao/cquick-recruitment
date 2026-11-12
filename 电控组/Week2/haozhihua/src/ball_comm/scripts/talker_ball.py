#!/usr/bin/env python3
import rclpy                      # ROS2 的 Python 主库
from rclpy.node import Node       # 所有节点都继承这个类
from ball_comm.msg import BallPos # 你自己定义的消息


class BallTalker(Node):
    def __init__(self):
        super().__init__('ball_talker')   # 节点名，ros2 node list 里会看到
        
        self.declare_parameter('rate_hz',2.0)
        hz = self.get_parameter('rate_hz').value

        # 创建发布者：消息类型 BallPos，话题名 ball_pos，队列长度 10
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        
        # 三个球的坐标（写死，不许改）
        self.balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]
        self.i = 0
        
        # 定时器
        self.timer = self.create_timer(1.0 / hz, self.tick)

    def tick(self):
        # 轮着取：0→1→2→0→1→2...
        ball_id, x, y = self.balls[self.i % 3]
        
        msg = BallPos()        # 造一个空消息
        msg.id = ball_id       # 填字段
        msg.x = x
        msg.y = y
        
        self.pub.publish(msg)  # 发出去
        self.get_logger().info(f'发布球 {ball_id}: ({x}, {y})')
        self.i += 1


def main():
    rclpy.init()               # 初始化 ROS2
    node = BallTalker()
    try:
        rclpy.spin(node)       # 死循环，等定时器/回调触发
    except KeyboardInterrupt:  # Ctrl+C 退出
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()