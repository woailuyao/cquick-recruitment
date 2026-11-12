import glob
import os

import rclpy
from rclpy.node import Node

from mine_interfaces.msg import TargetPosition
from mine_detector.detector import detect#方便得参数，省的全部复制



class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')

        img_dir = os.path.expanduser('~/vision_ws/mine/images')
        self.declare_parameter('image_dir', img_dir)
        img_dir = self.get_parameter('image_dir').value
        self.images = sorted(glob.glob(os.path.join(img_dir, '*.png')))
        self.idx = 0

        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)
        # 发布者：消息类型、话题名、队列长度
        self.create_timer(1.0, self.tick)
        # 定时器：每秒触发一次 tick

        self.get_logger().info(
                    f'发布节点启动：{len(self.images)} 张图，话题 /target_position')
        

    def tick(self):
        msg = TargetPosition()

        if not self.images:
            msg.detected = False
            msg.u = msg.v = msg.area = 0.0
            self.pub.publish(msg)
            self.get_logger().warn('没有图片可处理')
            return

        path = self.images[self.idx % len(self.images)]
        self.idx += 1

        detected, u, v, area = detect(path)

        msg.detected = bool(detected)
        msg.u = float(u)
        msg.v = float(v)
        msg.area = float(area)
        self.pub.publish(msg)

        if detected:
            self.get_logger().info(
            f'[{os.path.basename(path)}] u={u:.1f} v={v:.1f} area={area:.0f}')
        else:
            self.get_logger().warn(f'[{os.path.basename(path)}] 未检测到目标')
        


def main():
    rclpy.init()
    node=Publisher()
    (KeyboardInterrupt, Exception)
    try:
            rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()