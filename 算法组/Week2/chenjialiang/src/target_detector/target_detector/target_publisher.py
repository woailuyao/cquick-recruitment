import os

import cv2
import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition
from target_detector.detector import detect_image


class TargetPublisher(Node):
    def __init__(self):
        super().__init__("target_publisher")

        # 图片路径可以用命令行参数传，不传就用第一张球图
        default_img = os.path.expanduser("~/ros2_ws/data/ball1_clear.jpg")
        self.declare_parameter("image_path", default_img)
        self.image_path = self.get_parameter("image_path").value

        # 只读一次图，后面每次检测都复用这一张，不用重复读盘
        self.img = cv2.imread(self.image_path)
        if self.img is None:
            self.get_logger().error(f"读不到图片: {self.image_path}")
        else:
            self.get_logger().info(f"已载入 {self.image_path}  shape={self.img.shape}")

        # 往 /target_position 发 TargetPosition 类型的消息，队列长度 10
        self.pub = self.create_publisher(TargetPosition, "/target_position", 10)

        # 每 1 秒触发一次，相当于持续在检测（真实场景是每来一帧测一次）
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = TargetPosition()

        # 调检测函数，拿到当前的坐标和面积
        res = detect_image(self.img)

        # 没检测到也要发一帧，用 detected=False 告诉对方“这次没找到”，
        # 不能靠 u/v 是不是 0 来判断，因为 (0,0) 本身就是合法坐标
        if res is None:
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
            self.get_logger().warn("未检测到目标 -> detected=False")
        else:
            msg.detected = True
            msg.u = res["u"]
            msg.v = res["v"]
            msg.area = res["area"]
            self.get_logger().info(
                f"发布: detected=True u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f}"
            )

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)          # 初始化 ROS2
    node = TargetPublisher()
    try:
        rclpy.spin(node)           # 进入循环，等定时器触发回调
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()        # 退出前销毁节点、关闭 ROS2
        rclpy.shutdown()


if __name__ == "__main__":
    main()
