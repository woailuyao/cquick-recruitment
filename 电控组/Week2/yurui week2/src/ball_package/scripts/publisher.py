#!/usr/bin/env python3
import rclpy#ROS client library for python
from rclpy.node import Node
from ball_package.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        # 创建发布器，话题名 ball_pos，消息类型BallPos，队列长度10
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        timer_period = 0.5# 周期0.5秒，对应2Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # 3组球数据
        self.ball_data = [
            {"id": 1, "x": 2.0, "y": 3.0},
            {"id": 2, "x": 5.5, "y": 8.0},
            {"id": 3, "x": 9.0, "y": 4.0}
        ]
        self.index = 0

    def timer_callback(self):
        # 定时器回调函数，定时执行
        msg = BallPos()
        data = self.ball_data[self.index]
        msg.id = data["id"]
        msg.x = data["x"]
        msg.y = data["y"]
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布球信息：id={msg.id}, x={msg.x}, y={msg.y}')
        # 索引循环 0,1,2,0,1,2...
        self.index = (self.index + 1) % 3

def main(args=None):
    rclpy.init(args=args)
    ball_publisher = BallPublisher()
    rclpy.spin(ball_publisher)
    # 销毁节点，释放资源
    ball_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
