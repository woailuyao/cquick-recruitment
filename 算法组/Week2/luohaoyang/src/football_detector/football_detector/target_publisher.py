from pathlib import Path

import rclpy
from rclpy.node import Node

from cquick_interfaces.msg import TargetPosition
from football_detector.detection import detect_football


class TargetPublisher(Node):
    def __init__(self):
        super().__init__("target_publisher")
        self.publisher_ = self.create_publisher(
            TargetPosition, "/target_position", 10
        )
        self.timer = self.create_timer(1.0, self.publish_target)

        self.image_dir = Path(__file__).parent / "images"
        self.image_names = ["06.png", "07.png", "08.png"]
        self.index = 0
        self.count = 0

        self.get_logger().info(f"image directory: {self.image_dir}")

    def publish_target(self):
        name = self.image_names[self.index % len(self.image_names)]
        self.index += 1

        result = detect_football(self.image_dir / name)

        msg = TargetPosition()
        if result is None:
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
        else:
            msg.detected = True
            msg.u = result["u"]
            msg.v = result["v"]
            msg.area = result["area"]

        self.publisher_.publish(msg)
        self.count += 1
        self.get_logger().info(
            f"publish {self.count}: file={name}, "
            f"detected={msg.detected}, u={msg.u:.1f}, "
            f"v={msg.v:.1f}, area={msg.area:.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()