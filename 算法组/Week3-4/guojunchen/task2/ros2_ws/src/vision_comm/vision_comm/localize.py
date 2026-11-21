import os
import sys

import rclpy
from rclpy.node import Node
from vision_interfaces.msg import FieldPose, TargetPosition
import math

# 系统 python 只装了 rclpy（细则 2.3 禁止把深度学习依赖装进系统环境），
# 所以这里如果 import 不到 ultralytics，就自动换成项目 venv 的解释器重新执行自己。
_VENV_PY = os.environ.get(
    'VISION_VENV_PY',
    os.path.expanduser('~/vision_ws/k1_soccer_vision/.venv/bin/python'))
if not os.environ.get('_VISION_REEXEC') and os.path.exists(_VENV_PY):
    try:
        import ultralytics  # noqa: F401
    except Exception:
        os.environ['_VISION_REEXEC'] = '1'
        os.execv(_VENV_PY, [_VENV_PY, '-m', 'vision_comm.localize'] + sys.argv[1:])

from ultralytics import YOLO

# 项目根目录按三种方式确定，谁先命中用谁：
#   1) 环境变量 TASK1_DIR / TASK2_DIR（换机器、项目放过位置时用）
#   2) 从本文件位置向上推算（源码在 <项目>/task2/ros2_ws/src/vision_comm/vision_comm/ 时正确）
#   3) 常见安装路径兜底（把包装进 ~/ros2_ws/src 后第 2 条会失效，靠这里找工作区）
_THIS = os.path.dirname(os.path.abspath(__file__))

_CANDIDATES = [
    os.path.abspath(os.path.join(_THIS, '..', '..', '..', '..', '..')),  # 源码树里
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

sys.path.insert(0, os.path.join(TASK2_DIR, 'src'))
import live_localize as L


class LocalizerPublisher(Node):
    def __init__(self):
        super().__init__('localizer_publisher')
        self.pub_pose = self.create_publisher(FieldPose, '/field_pose', 10)
        self.pub_ball = self.create_publisher(TargetPosition, '/target_position', 10)
        self.declare_parameter('conf', 0.2)
        self.declare_parameter('r', 0.75)
        self.declare_parameter('init_side', 0)
        self.declare_parameter('source', '1')
        self.declare_parameter('publish_ball', True)   # 一台相机同时发两个话题，省得抢设备
        self.declare_parameter('conf_ball', 0.3)
        self.declare_parameter('min_area', 300)

        # 🌟 标志模型 + 足球模型 + 内参 + 场地点表
        self.mark_model = YOLO(os.path.join(TASK2_DIR, 'weights', 'marker_best.pt'))
        self.ball_model = YOLO(os.path.join(TASK1_DIR, 'weights', 'best.pt'))
        intr = os.path.join(TASK2_DIR, 'calibration', 'intrinsics.yaml')
        self.K = L.parse_intrinsics(intr)
        self.dist = L.parse_dist(intr)
        self.markers = L.field_markers(self.get_parameter('r').value)

        source = self.get_parameter('source').value
        self.cap = L.open_source(source)

        self.prev_pose = None
        self.heights = []

        self.timer = self.create_timer(1.0 / 15.0, self.publish_msg)
        self.get_logger().info(f'localizer_publisher 已启动，{len(self.markers)} 个地图点')

    def publish_msg(self):
        ok, img = self.cap.read()
        if not ok:
            return

        # ---------- 1. 场地位姿 ----------
        conf = self.get_parameter('conf').value
        res = self.mark_model.predict(img, imgsz=640, conf=conf, verbose=False)[0]

        raw = []
        for b in res.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            raw.append((int(b.cls[0]), (x1 + x2) / 2, (y1 + y2) / 2,
                        max(x2 - x1, y2 - y1) / 2, float(b.conf[0])))
        dets = L.refine_all(img, raw)

        prev_xy = None
        if self.prev_pose is not None:
            prev_xy = (self.prev_pose['x'], self.prev_pose['y'])

        pose, status, info = L.localize(
            dets, self.K, self.markers,
            prev_xy=prev_xy,
            prev_pose=self.prev_pose,
            dist=self.dist,
            init_side=self.get_parameter('init_side').value,
        )

        msg = FieldPose()
        msg.status = status
        msg.marker_count = int(info['n'])
        msg.residual = float(info['residual'] if info['residual'] else 0.0)

        if pose is not None:
            self.heights.append(pose['h'])
            self.prev_pose = pose
            msg.valid = True
            msg.x = float(pose['x'])
            msg.y = float(pose['y'])
            msg.yaw = float(math.degrees(pose['yaw']))
            self.get_logger().info(f'  -> 位姿 x={msg.x:+.2f}, y={msg.y:+.2f}, yaw={msg.yaw:+.1f}')
        else:
            msg.valid = False
            msg.x = msg.y = msg.yaw = 0.0
            self.get_logger().warn(f'  -> 未解出位姿: {status}')

        self.pub_pose.publish(msg)

        # ---------- 2. 足球（可选，同一帧） ----------
        if not self.get_parameter('publish_ball').value:
            return

        conf_ball = self.get_parameter('conf_ball').value
        min_area = self.get_parameter('min_area').value
        bres = self.ball_model.predict(img, imgsz=640, conf=conf_ball, verbose=False)[0]

        valid = []
        for b in bres.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            area = (x2 - x1) * (y2 - y1)
            if area > min_area:
                valid.append((area, (x1 + x2) / 2, (y1 + y2) / 2, float(b.conf[0])))

        # 面积从大到小排序
        valid = sorted(valid, key=lambda x: x[0], reverse=True)

        if len(valid) > 0:
            for i, (area, u, v, c) in enumerate(valid):
                bmsg = TargetPosition()
                bmsg.detected = True
                bmsg.u = float(int(u))
                bmsg.v = float(int(v))
                bmsg.area = float(area)
                bmsg.confidence = c
                self.pub_ball.publish(bmsg)
                self.get_logger().info(f'  -> 发送目标 {i + 1}: u={bmsg.u}, v={bmsg.v}')
        else:
            bmsg = TargetPosition()
            bmsg.detected = False
            bmsg.u = bmsg.v = bmsg.area = 0.0
            self.pub_ball.publish(bmsg)
            self.get_logger().warn('  -> 该帧未检测到目标')


def main():
    rclpy.init()
    rclpy.spin(LocalizerPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
