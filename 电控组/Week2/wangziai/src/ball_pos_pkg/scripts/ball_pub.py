#!/usr/bin/env python3
"""发布者：2 Hz 循环发布 3 个球的坐标（BallPos）。"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from ball_pos_pkg.msg import BallPos

# 规定的 3 个球坐标
BALLS = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]

# 继承ROS2节点Node
class BallPublisher(Node):
    def __init__(self):
        # 注册节点名
        super().__init__('ball_publisher')

        # 创建发布者（消息类型、话题名、QoS 队列深度）
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)

        # 设置发布速率（声明参数：默认 2.0 Hz，运行时可用 ros2 param set 改）
        self.declare_parameter('publish_rate', 2.0)

        # 初始化定时器
        # 先置空：后面重建定时器时要先判断它存不存在
        self.timer = None
        # 启动时先按参数里的频率建一个定时器（不建的话节点起来不会发消息）
        # 用 get_parameter_value().double_value 取值，类型明确是 float，不会报类型错误
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').get_parameter_value().double_value,
            self.on_timer)
        # 注册参数回调：每次 ros2 param set 改 publish_rate 都会进 set_timer
        self.add_on_set_parameters_callback(self.set_timer)

    # 参数改变时重建定时器，让新频率立刻生效
    def set_timer(self, params):
        # params 是这次被修改的参数列表（一次可能改多个，要逐个检查）
        for p in params:
            if p.name == 'publish_rate':
                # 频率必须大于 0，否则 1.0 / 0 会让节点崩掉
                if p.value <= 0:
                    self.get_logger().info("速度不得为负")
                    # 返回失败结果，ros2 param set 会提示“被拒绝”
                    return SetParametersResult(successful=False, reason='频率必须大于 0')
                else:
                    # 先销毁旧定时器（不销毁的话每改一次多一个，频率会叠加）
                    if self.timer is not None:
                        self.destroy_timer(self.timer)
                    # 用新频率重建定时器
                    self.timer = self.create_timer(1.0 / p.value, self.on_timer)
                    # 打印确认，改频率截图时用得上
                    self.get_logger().info(f"已设置发布速度为 {p.value} Hz, 发布频率为 {1 / p.value} 秒每条")
        # 所有参数检查通过，本次设置生效
        return SetParametersResult(successful=True)

    # 发布函数：定时器每个周期调用一次，把 3 个球各发一遍
    def on_timer(self):
        # 依次发布球
        for id, x, y in BALLS:
            # 构造消息
            msg = BallPos(id=id, x=x, y=y)
            # 发布到话题 ball_pos
            self.pub.publish(msg)
            # 打印发了什么，方便和订阅端对照
            self.get_logger().info(f"已发布：球id {id}, 坐标 ({x}, {y})")

# 主程序
def main(args=None):
    # 脚本初始化
    rclpy.init(args=args)

    # 建立发布节点
    node = BallPublisher()

    # 发布节点发布内容
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    # 脚本安全结束
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
