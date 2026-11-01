"""Log received target positions, including invalid message diagnostics."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from target_interfaces.msg import TargetPosition


def _status(message):
    fields = (message.u, message.v, message.area)
    if not all(math.isfinite(value) for value in fields):
        return 'invalid', 'u, v, and area must be finite'
    if any(value < 0 for value in fields):
        return 'invalid', 'u, v, and area must be nonnegative'
    if message.detected:
        if message.area <= 0:
            return 'invalid', 'detected=True requires positive area'
        return 'detected', ''
    if any(value != 0 for value in fields):
        return 'invalid', 'detected=False requires zero u, v, and area'
    return 'no_target', ''


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self._count = 0
        self._invalid_count = 0
        self._subscription = self.create_subscription(
            TargetPosition, '/target_position', self._receive, 10
        )

    def _receive(self, message):
        self._count += 1
        status, reason = _status(message)
        if status == 'invalid':
            self._invalid_count += 1
        text = (
            f'count={self._count} detected={message.detected} '
            f'u={message.u:.6f} v={message.v:.6f} area={message.area:.6f} '
            f'status={status} invalid_count={self._invalid_count}'
        )
        if reason:
            self.get_logger().warning(f'{text} reason={reason}')
        else:
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetSubscriber()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
