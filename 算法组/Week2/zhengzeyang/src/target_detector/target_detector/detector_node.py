import glob
import os

import cv2
import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition
from target_detector.detector_core import detect_target


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.declare_parameter(
            'image_dir', os.path.expanduser('~/vision_ws/week2/images'))
        self.declare_parameter('image_path', '')
        self.declare_parameter('rate', 1.0)

        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)
        rate = max(float(self.get_parameter('rate').value), 0.1)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.idx = 0
        self.get_logger().info('detector_node ready, rate=%.2f Hz' % rate)

    def tick(self):
        msg = TargetPosition()
        path = self.get_parameter('image_path').value

        if not path:
            d = self.get_parameter('image_dir').value
            files = sorted(glob.glob(os.path.join(d, '*.jpg')))
            if not files:
                self.get_logger().warn('no jpg in %s' % d)
                msg.detected = False
                self.pub.publish(msg)
                return
            path = files[self.idx % len(files)]
            self.idx += 1

        img = cv2.imread(path)
        name = os.path.basename(path)

        if img is None:
            self.get_logger().warn('cannot read %s' % path)
            msg.detected = False
            self.pub.publish(msg)
            return

        r = detect_target(img)
        msg.detected = bool(r['found'])
        msg.u = float(r['u'])
        msg.v = float(r['v'])
        msg.area = float(r['area'])
        self.pub.publish(msg)

        if r['found']:
            self.get_logger().info(
                'pub %-12s u=%.1f v=%.1f area=%.0f' % (name, msg.u, msg.v, msg.area))
        else:
            self.get_logger().info('pub %-12s detected=False' % name)


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

