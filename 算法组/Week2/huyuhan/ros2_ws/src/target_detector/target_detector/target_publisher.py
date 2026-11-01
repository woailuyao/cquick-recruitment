from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition


def detect_target(image):
    msg = TargetPosition()
    msg.detected = False
    msg.u = 0.0
    msg.v = 0.0
    msg.area = 0.0

    result = image.copy()
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    lower = np.array([20, 40, 50], dtype=np.uint8)
    upper = np.array([45, 255, 255], dtype=np.uint8)
    mask = cv.inRange(hsv, lower, upper)

    contours, _ = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
    )
    contours = sorted(contours, key=cv.contourArea, reverse=True)

    for cnt in contours:
        area = cv.contourArea(cnt)
        if area < 500:
            continue

        x, y, w, h = cv.boundingRect(cnt)
        if not 0.7 <= w / h <= 1.3:
            continue

        perimeter = cv.arcLength(cnt, True)
        if perimeter == 0:
            continue

        circularity = 4 * np.pi * area / perimeter**2
        if circularity < 0.6:
            continue

        msg.detected = True
        msg.u = float(x + w / 2)
        msg.v = float(y + h / 2)
        msg.area = float(area)

        cv.drawContours(result, [cnt], -1, (0, 0, 255), 2)
        cv.circle(
            result, (round(msg.u), round(msg.v)),
            4, (0, 255, 0), -1
        )
        cv.putText(
            result, f"u={msg.u:.1f}, v={msg.v:.1f}",
            (x, y + h + 20),
            cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
        )
        break

    if not msg.detected:
        cv.putText(
            result, "No target detected", (20, 40),
            cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
        )

    return msg, result


class TargetPublisher(Node):
    def __init__(self):
        super().__init__("target_publisher")

        print("姓名：胡钰涵 / Hu Yuhan", flush=True)
        print(
            "运行时间："
            + datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y-%m-%d %H:%M:%S"),
            flush=True
        )

        self.declare_parameter(
            "image_path", "images/clear.jpg"
        )
        image_path = Path(self.get_parameter("image_path").value).expanduser()
        self.get_logger().info(f"输入图片：{image_path}")

        image = cv.imdecode(
            np.fromfile(str(image_path), dtype=np.uint8), cv.IMREAD_COLOR
        )
        if image is None:
            raise FileNotFoundError(f"图片读取失败：{image_path}")

        self.msg, result = detect_target(image)

        output_dir = Path(image_path).parent.parent / "results"
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / (
            Path(image_path).stem + "_ros2_detected.jpg"
        )
        ok, encoded = cv.imencode(
            ".jpg", result, [cv.IMWRITE_JPEG_QUALITY, 90]
        )
        if not ok:
            raise OSError(f"结果保存失败：{output_path}")
        encoded.tofile(str(output_path))

        self.get_logger().info(f"标注图已保存：{output_path}")

        self.publisher_ = self.create_publisher(
            TargetPosition, "/target_position", 10
        )
        self.count = 0
        self.timer = self.create_timer(1.0, self.publish_target)

    def publish_target(self):
        self.publisher_.publish(self.msg)
        self.count += 1
        self.get_logger().info(
            f"发布第{self.count}条：detected={self.msg.detected}, "
            f"u={self.msg.u:.1f}, v={self.msg.v:.1f}, "
            f"area={self.msg.area:.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
