import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from target_interfaces.msg import TargetPosition
from target_publisher.detect import imread_zh, detect

IMG_DIR = os.path.join(get_package_share_directory("target_publisher"), "images")
NAMES = ["test4", "test5", "test6", "test7", "test8"]


class TargetPublisher(Node):
    def __init__(self):
        super().__init__("target_publisher")
        self.declare_parameter("h_min", 8)
        self.declare_parameter("h_max", 25)
        self.declare_parameter("s_min", 120)
        self.declare_parameter("s_max", 255)
        self.declare_parameter("v_min", 80)
        self.declare_parameter("v_max", 255)
        self.declare_parameter("min_area", 500)
        self.declare_parameter("min_ratio", 0.5)
        self.declare_parameter("ksize", 5)
        self.pub = self.create_publisher(TargetPosition, "/target_position", 10)
        self.idx = 0
        self.timer = self.create_timer(1.0, self.tick)

    def get_args(self):
        p = self.get_parameter
        lo = (p("h_min").value, p("s_min").value, p("v_min").value)
        hi = (p("h_max").value, p("s_max").value, p("v_max").value)
        return lo, hi, p("min_area").value, p("min_ratio").value, p("ksize").value

    def tick(self):
        name = NAMES[self.idx % len(NAMES)]
        self.idx += 1
        lo, hi, min_area, min_ratio, ksize = self.get_args()
        msg = TargetPosition()
        img = imread_zh(os.path.join(IMG_DIR, name + ".png"))
        if img is None:
            msg.detected = False
            msg.u = -1.0
            msg.v = -1.0
            msg.area = 0.0
        else:
            detected, u, v, area, c, mask = detect(img, lo, hi, min_area, min_ratio, ksize)
            msg.detected = bool(detected)
            msg.u = float(u)
            msg.v = float(v)
            msg.area = float(area)
        self.pub.publish(msg)
        self.get_logger().info("%s detected=%s u=%.1f v=%.1f area=%.0f" % (name, msg.detected, msg.u, msg.v, msg.area))


def main():
    rclpy.init()
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
