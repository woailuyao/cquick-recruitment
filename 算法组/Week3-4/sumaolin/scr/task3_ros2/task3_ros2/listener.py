#!/usr/bin/env python3
"""订阅 /task3/localization 并打印，用于验证接口。"""

import rclpy
from rclpy.node import Node

from task3_interfaces.msg import Localization

TOPIC = "/task3/localization"


class Task3Listener(Node):
    def __init__(self):
        super().__init__("task3_listener")
        self.sub = self.create_subscription(Localization, TOPIC, self.callback, 10)
        self.count = 0
        self.get_logger().info(f"listening on {TOPIC}")

    def callback(self, msg):
        self.count += 1
        marker_types = ",".join(item.type for item in msg.markers)
        self.get_logger().info(
            f"[{self.count:04d}] valid={msg.valid} temporal={msg.temporal} "
            f"status={msg.status} "
            f"pose=({msg.x_camera_map:.2f}, {msg.y_camera_map:.2f}, "
            f"{msg.yaw_camera_map:.1f}) markers={marker_types} balls={len(msg.balls)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Task3Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
