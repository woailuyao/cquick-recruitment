import os
import sys

import rclpy
from rclpy.node import Node
from vision_interfaces.msg import TargetPosition
import cv2

# 原因同 localize.py：系统 python 没有 ultralytics，自动换 venv 解释器重跑
_VENV_PY = os.environ.get(
    'VISION_VENV_PY',
    os.path.expanduser('~/vision_ws/k1_soccer_vision/.venv/bin/python'))
if not os.environ.get('_VISION_REEXEC') and os.path.exists(_VENV_PY):
    try:
        import ultralytics  # noqa: F401
    except Exception:
        os.environ['_VISION_REEXEC'] = '1'
        os.execv(_VENV_PY, [_VENV_PY, '-m', 'vision_comm.pub'] + sys.argv[1:])

from ultralytics import YOLO

# 默认值按本文件位置推算，Windows / Linux 都能用；也可用环境变量 TASK1_DIR / TASK2_DIR 覆盖
_THIS = os.path.dirname(os.path.abspath(__file__))
_CANDIDATES = [
    os.path.abspath(os.path.join(_THIS, '..', '..', '..', '..', '..')),
    '/mnt/d/guojunchen',
    os.path.expanduser('~/guojunchen'),
    '/opt/guojunchen',
    'D:/guojunchen',
]


def _pick_root():
    for c in _CANDIDATES:
        if os.path.isdir(os.path.join(c, 'task2', 'src')) and \
                os.path.isfile(os.path.join(c, 'task2', 'src', 'live_localize.py')):
            return c
    return _CANDIDATES[0]


_GUOJUNCHEN = _pick_root()
TASK2_DIR = os.environ.get('TASK2_DIR', os.path.join(_GUOJUNCHEN, 'task2'))
TASK1_DIR = os.environ.get('TASK1_DIR', os.path.join(_GUOJUNCHEN, 'task1'))


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)
        self.declare_parameter('conf', 0.3)
        self.declare_parameter('min_area', 300)
        self.declare_parameter('source', '1')

        self.model = YOLO(os.path.join(TASK1_DIR, 'weights', 'best.pt'))

        # 🌟 相机：1 是 USB 单目（给视频路径也能跑）
        source = self.get_parameter('source').value
        self.cap = cv2.VideoCapture(int(source) if str(source).isdigit() else source)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.timer = self.create_timer(1.0 / 30.0, self.publish_msg)
        self.get_logger().info('target_publisher 已启动，发布 /target_position')

    def publish_msg(self):
        ok, img = self.cap.read()
        if not ok:
            return

        conf = self.get_parameter('conf').value
        min_area = self.get_parameter('min_area').value

        res = self.model.predict(img, imgsz=640, conf=conf, verbose=False)[0]

        # 面积筛选：和 task1 一样，把太小的噪点滤掉
        valid = []
        for b in res.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            area = (x2 - x1) * (y2 - y1)
            if area > min_area:
                valid.append((area, (x1 + x2) / 2, (y1 + y2) / 2, float(b.conf[0])))

        if len(valid) > 0:
            # 面积从大到小排序
            valid = sorted(valid, key=lambda x: x[0], reverse=True)
            for i, (area, u, v, c) in enumerate(valid):
                msg = TargetPosition()
                msg.detected = True
                msg.u = float(int(u))
                msg.v = float(int(v))
                msg.area = float(area)
                msg.confidence = c

                self.pub.publish(msg)
                self.get_logger().info(f'  -> 发送目标 {i+1}: u={msg.u}, v={msg.v}')
        else:
            msg = TargetPosition()
            msg.detected = False
            msg.u = msg.v = msg.area = 0.0
            self.pub.publish(msg)
            self.get_logger().warn('  -> 该帧未检测到目标')


def main():
    rclpy.init()
    rclpy.spin(TargetPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
