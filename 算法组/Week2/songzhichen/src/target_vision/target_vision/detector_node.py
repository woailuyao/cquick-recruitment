# -*- coding: utf-8 -*-
"""发布节点：读测试图片做 HSV 检测，把结果发到 /target_position。

对应任务 3 的发布端要求：
  - 话题名 /target_position，类型 target_interfaces/msg/TargetPosition
  - 发布数据来自任务 1 的检测流程，不是写死的坐标
  - 没检测到目标时 detected=False，u/v/area 置 0，不能假装目标在 (0,0)

数据来源：按固定频率逐张循环处理 share/images 下的 5 张测试图片，
每处理一张发一条消息。这样不用摄像头也能验证整条通信链路。
"""
import glob
import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from target_interfaces.msg import TargetPosition
from target_vision.detector import detect_target


class DetectorNode(Node):

    def __init__(self):
        super().__init__('target_detector')

        # ---- 阈值做成 ROS2 参数，可以运行时改（细则加分项）----
        self.declare_parameter('lower_h', 5)
        self.declare_parameter('lower_s', 120)
        self.declare_parameter('lower_v', 120)
        self.declare_parameter('upper_h', 25)
        self.declare_parameter('upper_s', 255)
        self.declare_parameter('upper_v', 255)
        self.declare_parameter('min_area', 800)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('image_dir', '')   # 留空则用包内自带的 images

        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)

        image_dir = self.get_parameter('image_dir').value
        if not image_dir:
            image_dir = os.path.join(
                get_package_share_directory('target_vision'), 'images')
        self.image_dir = image_dir
        self.images = sorted(glob.glob(os.path.join(image_dir, '*.jpg')))
        if not self.images:
            self.get_logger().error('图片目录里没有 jpg: %s' % image_dir)
        else:
            self.get_logger().info('共 %d 张测试图片，目录: %s'
                                   % (len(self.images), image_dir))

        self.idx = 0
        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.tick)
        self.get_logger().info('发布节点已启动，话题 /target_position，频率 %.1f Hz' % rate)

    def params(self):
        gp = self.get_parameter
        lower = (gp('lower_h').value, gp('lower_s').value, gp('lower_v').value)
        upper = (gp('upper_h').value, gp('upper_s').value, gp('upper_v').value)
        return lower, upper, int(gp('min_area').value)

    def tick(self):
        if not self.images:
            return
        path = self.images[self.idx % len(self.images)]
        self.idx += 1

        img = cv2.imread(path)
        msg = TargetPosition()
        if img is None:
            # 读不到图也算「本帧未检测到」，不要发假坐标
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
            self.pub.publish(msg)
            self.get_logger().warn('读不到图片: %s' % path)
            return

        lower, upper, min_area = self.params()
        res = detect_target(img, lower, upper, min_area)

        if res['detected']:
            msg.detected = True
            msg.u = float(res['u'])
            msg.v = float(res['v'])
            msg.area = float(res['area'])
            self.get_logger().info(
                '发第 %d 条 | %s | detected=True u=%.1f v=%.1f area=%.0f'
                % (self.idx, os.path.basename(path), msg.u, msg.v, msg.area))
        else:
            # 关键：未检测到时字段必须是 0，靠 detected 判断本帧有效
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
            self.get_logger().info(
                '发第 %d 条 | %s | detected=False u=0.0 v=0.0 area=0.0（本帧未检测到目标）'
                % (self.idx, os.path.basename(path)))

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
