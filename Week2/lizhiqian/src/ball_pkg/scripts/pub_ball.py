#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos
from rcl_interfaces.msg import SetParametersResult

class BallPubNode(Node):
    def __init__(self):
        super().__init__("ball_pub")
        self.pub = self.create_publisher(BallPos, "ball_pos", 10)

        # 声明参数
        self.declare_parameter("timer_period", 0.2)
        self.timer_period = self.get_parameter("timer_period").get_parameter_value().double_value

        # 创建定时器
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # 注册参数回调！核心，用来监听参数变化
        self.add_on_set_parameters_callback(self.param_callback)

        self.ball_list = [
            {"id":1, "x":2.0, "y":3.0},
            {"id":2, "x":5.5, "y":8.0},
            {"id":3, "x":9.0, "y":4.0},
        ]
        self.index = 0

    # 参数变化回调函数
    def param_callback(self, params):
        for p in params:
            if p.name == "timer_period":
                self.timer_period = p.value
                # 销毁旧定时器，新建定时器 → 频率立刻更新
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(self.timer_period, self.timer_cb)
                self.get_logger().info(f"定时器周期更新为：{self.timer_period} s")
        return SetParametersResult(successful=True)

    def timer_cb(self):
        b = self.ball_list[self.index]
        msg = BallPos()
        msg.id = b["id"]
        msg.x = b["x"]
        msg.y = b["y"]
        self.pub.publish(msg)
        self.get_logger().info(f"发布球  id={msg.id}, x={msg.x}, y={msg.y}")
        self.index = (self.index + 1) % len(self.ball_list)

def main(args=None):
    rclpy.init(args=args)
    node = BallPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()