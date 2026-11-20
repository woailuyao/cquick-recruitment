import os

import rclpy
from rclpy.node import Node

from cquick_color_vision.core import (
    DetectorConfig,
    detect_target,
    read_image,
)
from cquick_vision_msgs.msg import TargetPosition


class TargetPublisher(Node):
    def __init__(self) -> None:
        super().__init__("target_position_publisher")

        self.declare_parameter(
            "image_path",
            "test_images/clear.png",
        )
        self.declare_parameter("publish_period", 0.5)

        self.declare_parameter("h_min", 5)
        self.declare_parameter("h_max", 25)
        self.declare_parameter("s_min", 100)
        self.declare_parameter("s_max", 255)
        self.declare_parameter("v_min", 70)
        self.declare_parameter("v_max", 255)

        self.declare_parameter("min_area", 800)
        self.declare_parameter("kernel_size", 5)
        self.declare_parameter(
            "morph_mode",
            "open_close",
        )

        self.publisher = self.create_publisher(
            TargetPosition,
            "/target_position",
            10,
        )

        period = float(
            self.get_parameter("publish_period").value
        )

        self.timer = self.create_timer(
            max(period, 0.05),
            self.publish_detection,
        )

        self.message_count = 0

        self.get_logger().info(
            "目标位置发布节点已启动，话题: /target_position"
        )

    def current_config(self) -> DetectorConfig:
        value = lambda name: self.get_parameter(name).value

        config = DetectorConfig(
            h_min=int(value("h_min")),
            h_max=int(value("h_max")),
            s_min=int(value("s_min")),
            s_max=int(value("s_max")),
            v_min=int(value("v_min")),
            v_max=int(value("v_max")),
            min_area=int(value("min_area")),
            kernel_size=int(value("kernel_size")),
            morph_mode=str(value("morph_mode")),
        )

        config.validate()
        return config

    def publish_detection(self) -> None:
        image_value = str(
            self.get_parameter("image_path").value
        )

        image_path = os.path.abspath(
            os.path.expanduser(image_value)
        )

        image = read_image(image_path)
        msg = TargetPosition()

        if image is None:
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0

            self.get_logger().error(
                "图片读取失败，发布 detected=false: "
                f"{image_path}"
            )
        else:
            try:
                detection = detect_target(
                    image,
                    self.current_config(),
                )
            except ValueError as error:
                self.get_logger().error(
                    f"参数错误，发布 detected=false: {error}"
                )
                detection = None

            if detection is not None and detection.detected:
                msg.detected = True
                msg.u = float(detection.u)
                msg.v = float(detection.v)
                msg.area = float(detection.area)

                self.get_logger().info(
                    f"第 {self.message_count + 1} 条: "
                    f"detected={msg.detected}, "
                    f"u={msg.u:.2f}, "
                    f"v={msg.v:.2f}, "
                    f"area={msg.area:.2f}"
                )
            else:
                msg.detected = False
                msg.u = 0.0
                msg.v = 0.0
                msg.area = 0.0

                self.get_logger().info(
                    f"第 {self.message_count + 1} 条: "
                    "detected=false，未检测到目标"
                )

        self.publisher.publish(msg)
        self.message_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
