"""Publish detections from real image files on /target_position."""

import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from target_interfaces.msg import TargetPosition

from .detector import DetectorConfig, detect


_DYNAMIC = ('hsv_lower', 'hsv_upper', 'min_area', 'morphology', 'kernel_size')
_STATIC = frozenset(('images', 'frequency', 'max_messages'))
_FLOAT32_MAX = float(np.finfo(np.float32).max)


def _configuration(values):
    """Let DetectorConfig validate the entire proposed parameter transaction."""
    return DetectorConfig(**{name: values[name] for name in _DYNAMIC})


def _read_images(paths):
    """Decode all paths once, including Unicode paths; errors abort startup."""
    if not paths or any(not isinstance(path, str) or not path.strip() for path in paths):
        raise ValueError('images must be a nonempty string array of real image paths')
    images = []
    for name in paths:
        path = Path(name).expanduser()
        if not path.is_file():
            raise ValueError(f'image is not a regular file: {path}')
        try:
            encoded = np.fromfile(str(path), dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR) if encoded.size else None
        except (OSError, ValueError, cv2.error) as exc:
            raise ValueError(f'failed to read image {path}: {exc}') from exc
        if image is None or image.size == 0:
            raise ValueError(f'failed to decode image: {path}')
        images.append((str(path), image))
    return images


class TargetPublisher(Node):
    """Cycle through validated input images and run detection at each tick.

    Static parameters: images (string array), frequency (Hz), max_messages
    (zero means unlimited). Detection parameters may be changed atomically.
    """

    def __init__(self):
        super().__init__('target_publisher')
        self.declare_parameter('images', [''])
        self.declare_parameter('frequency', 2.0)
        self.declare_parameter('max_messages', 0)
        self.declare_parameter('hsv_lower', [5, 100, 80])
        self.declare_parameter('hsv_upper', [25, 255, 255])
        self.declare_parameter('min_area', 100.0)
        self.declare_parameter('morphology', 'none')
        self.declare_parameter('kernel_size', 3)
        try:
            frequency = self.get_parameter('frequency').value
            if isinstance(frequency, bool) or not isinstance(frequency, (int, float)):
                raise ValueError('frequency must be a finite positive number')
            if not math.isfinite(frequency) or frequency <= 0:
                raise ValueError('frequency must be a finite positive number')
            self._max_messages = self.get_parameter('max_messages').value
            if type(self._max_messages) is not int or self._max_messages < 0:
                raise ValueError('max_messages must be an integer >= 0')
            self._values = {name: self.get_parameter(name).value for name in _DYNAMIC}
            self._config = _configuration(self._values)
            self._images = _read_images(self.get_parameter('images').value)
            self._publisher = self.create_publisher(TargetPosition, '/target_position', 10)
            self._count = 0
            self._complete = False
            self.add_on_set_parameters_callback(self._validate_parameters)
            self._timer = self.create_timer(1.0 / frequency, self._publish_next)
        except Exception:
            self.destroy_node()
            raise
        self.get_logger().info(
            f'Loaded {len(self._images)} image(s); frequency={frequency:g} Hz; '
            f'max_messages={self._max_messages} (0=unbounded)'
        )

    @property
    def complete(self):
        return self._complete

    def _validate_parameters(self, parameters):
        """Reject a whole transaction if any proposed value is invalid."""
        candidate = dict(self._values)
        for parameter in parameters:
            if parameter.name in _STATIC:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} is static; restart the node to change it',
                )
            if parameter.name in _DYNAMIC:
                candidate[parameter.name] = parameter.value
        try:
            config = _configuration(candidate)
        except (TypeError, ValueError, OverflowError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        self._values = candidate
        self._config = config
        return SetParametersResult(successful=True)

    def _publish_next(self):
        path, image = self._images[self._count % len(self._images)]
        # Do not cache detections: each tick uses the current detector settings.
        # Errors propagate; failed processing must never become a false no-target.
        result = detect(image, self._config)
        fields = (float(result.u), float(result.v), float(result.area))
        if any(not math.isfinite(value) or abs(value) > _FLOAT32_MAX for value in fields):
            raise ValueError(f'detector returned non-finite or non-float32 fields for {path}')
        message = TargetPosition()
        message.detected = bool(result.detected)
        message.u, message.v, message.area = fields
        self._publisher.publish(message)
        self._count += 1
        self.get_logger().info(
            f'count={self._count} detected={message.detected} '
            f'u={message.u:.6f} v={message.v:.6f} area={message.area:.6f} '
            f'image={path}'
        )
        if self._max_messages and self._count >= self._max_messages:
            self._timer.cancel()
            self._complete = True
            self.get_logger().info(f'Completed max_messages={self._max_messages}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetPublisher()
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        rclpy.logging.get_logger('target_publisher').fatal(str(exc))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
