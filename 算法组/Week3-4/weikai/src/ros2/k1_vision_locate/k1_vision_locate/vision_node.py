#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vision_node.py —— CQuick 第3-4周 ROS2 视觉定位节点（一个节点、两个话题）

链路：图像源 → YOLO 足球检测 + YOLO L/T 标志检测 → 固定外参地面投影
      → 2D Kabsch 地图配准 → (x, y, yaw) + 残差 + 有效状态

发布：
  /soccer/ball             cquick_interfaces/msg/BallDetection   （任务 1）
  /localization/field_pose cquick_interfaces/msg/FieldPose       （任务 2）

图像源三选一（--ros-args -p source:=...）：
  net://127.0.0.1:8090        Windows win_cam_bridge 的 MJPEG 流（实时，默认）
  /path/to/video.mp4          视频文件（回放/自测，读完即退出）
  0 / 1 / ...                 本机 V4L2 相机序号

依赖说明：ROS2 系统环境不装 PyTorch/Ultralytics（细则 2.3 约定），本节点
**必须用视觉 venv 的解释器运行**（见 demo_launch / README 运行命令）：
先 source ROS 环境获得 rclpy 的 PYTHONPATH 与动态库路径，再以
`~/vision_ws/k1_soccer_vision/.venv/bin/python` 启动——ultralytics/torch
来自 venv，rclpy/接口包来自 ROS 环境（同为 Python 3.10，兼容）。
不要把 venv site-packages 注入 ROS 系统 python 的 sys.path（会遮蔽
系统 sympy/numpy 导致 rclpy 无法工作，已实测踩坑）。

复用说明：检测与解算核心直接 import 仓库 src/ 下的 realtime_locate.py、
locator.py、locator_fixed.py、net_camera.py（与离线验证同一套代码路径，
保证 ROS 输出与录屏演示结果一致），不复制逻辑。
"""
import os
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node

from cquick_interfaces.msg import BallDetection, FieldPose
from std_msgs.msg import Header


class VisionNode(Node):

    def __init__(self):
        super().__init__("k1_vision_node")

        # ---------------- 参数 ----------------
        self.declare_parameter("source", "net://127.0.0.1:8090")
        self.declare_parameter(
            "ball_weights",
            "/home/weikai/vision_ws/k1_soccer_vision/runs/"
            "ball_yolov8n_aug2/weights/best.pt")
        self.declare_parameter(
            "marker_weights",
            "/mnt/d/captures/landmarks/weights/best_fixed_v2.pt")
        # 仓库 src 目录（含 realtime_locate.py / locator.py / locator_fixed.py）
        # realpath 解析 colcon --symlink-install 的软链，确保定位回源码目录
        pkg_dir = os.path.dirname(os.path.realpath(os.path.abspath(__file__)))
        default_repo_src = os.path.normpath(
            os.path.join(pkg_dir, "..", "..", ".."))
        self.declare_parameter("repo_src", default_repo_src)
        self.declare_parameter("field_yaml", "")
        self.declare_parameter("intrinsics_yaml",
                               "/mnt/d/captures/calib/intrinsics.yaml")
        self.declare_parameter("extrinsics_yaml", "")
        self.declare_parameter("ball_conf", 0.25)
        self.declare_parameter("marker_conf", 0.05)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("enhance", "none")
        self.declare_parameter("solve_stride", 5)
        self.declare_parameter("residual_max", 0.30)
        self.declare_parameter("smooth", 5)

        p = lambda name: self.get_parameter(name).value
        source = p("source")
        ball_w = p("ball_weights")
        marker_w = p("marker_weights")
        repo_src = p("repo_src")

        # 仓库 src 加入 sys.path（复用离线验证的同一套实现）
        if repo_src and os.path.isdir(repo_src):
            sys.path.insert(0, repo_src)

        from realtime_locate import get_model, detect_boxes  # noqa: E402
        from locator import load_field, load_intrinsics, field_type  # noqa: E402
        from locator_fixed import (  # noqa: E402
            load_extrinsics, camera_to_base, pixels_to_ground,
            solve_pose_fixed)
        self._detect_boxes = detect_boxes
        self._get_model = get_model
        self._pixels_to_ground = pixels_to_ground
        self._solve_pose_fixed = solve_pose_fixed

        import cv2  # noqa: E402
        import numpy as np  # noqa: E402
        self._cv2, self._np = cv2, np

        # ---------------- 发布器 ----------------
        self.pub_ball = self.create_publisher(BallDetection, "/soccer/ball", 10)
        self.pub_pose = self.create_publisher(FieldPose,
                                              "/localization/field_pose", 10)

        # 探针模式：只建 publisher 就 spin（用于诊断 DDS 图可见性）
        if os.environ.get("VNODE_PROBE") == "1":
            self.get_logger().info("PROBE: publishers created")

            def probe_tick():
                m = FieldPose()
                m.header = Header()
                m.header.frame_id = "map"
                m.status = "PROBE"
                self.pub_pose.publish(m)
            self.timer = self.create_timer(0.2, probe_tick)
            self.get_logger().info("PROBE ready")
            return

        # ---------------- 模型与标定 ----------------
        pkg_src = repo_src
        field_yaml = p("field_yaml") or os.path.join(
            pkg_src, "templates", "field.yaml")
        extr_yaml = p("extrinsics_yaml") or os.path.join(
            pkg_src, "templates", "camera_extrinsics.yaml")
        intr_yaml = p("intrinsics_yaml")

        self.field = load_field(field_yaml)
        self.K, self.dist, _ = load_intrinsics(intr_yaml)
        self.h, self.pitch, self.roll = load_extrinsics(extr_yaml)
        self.R_cb = camera_to_base(self.pitch, self.roll)
        self.map_names = [k for k in self.field
                          if field_type(k) is not None]
        self.map_pts = self._np.array([self.field[k] for k in self.map_names],
                                      float)
        self.get_logger().info(
            "固定外参 h=%.3fm pitch=%.1f° roll=%.1f° | 地图标志 %d 个"
            % (self.h, self.pitch, self.roll, len(self.map_names)))

        self.ball_model = self._get_model(ball_w)
        self.marker_model = self._get_model(marker_w)

        # ---------------- 图像源 ----------------
        self.is_file = False
        if source.startswith("net://"):
            from net_camera import NetCamera
            self.cap = NetCamera("http://" + source[6:])
        elif source.isdigit():
            self.cap = self._cv2.VideoCapture(int(source))
        else:
            self.cap = self._cv2.VideoCapture(source)
            self.is_file = True
        if not self.cap.isOpened():
            raise RuntimeError("打不开图像源: %s" % source)

        # ---------------- 状态 ----------------
        self.seq = 0
        self.stride = max(1, int(p("solve_stride")))
        self.residual_max = float(p("residual_max"))
        self.hist = deque(maxlen=max(1, int(p("smooth"))))  # (x, y, yaw)
        self.rms = None
        self.valid = False
        self.status = "INIT"
        self.marker_count = 0
        self._t_last = time.time()
        self._fps = 0.0
        self._t0 = time.time()

        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info("k1_vision_node 就绪，图像源: %s" % source)

    # ------------------------------------------------------------------
    def tick(self):
        cv2, np = self._cv2, self._np
        ok, frame = self.cap.read()
        if not ok:
            if self.is_file:
                self.get_logger().info("视频文件读完，共处理 %d 帧，退出"
                                       % self.seq)
                raise SystemExit(0)
            return  # 实时流短暂断帧：下一轮重试
        self.seq += 1
        if self.seq % 30 == 0:
            self.get_logger().info("已处理 %d 帧，fps=%.1f"
                                   % (self.seq, self._fps))

        # 帧率
        now = time.time()
        dt = now - self._t_last
        if dt > 0:
            inst = 1.0 / dt
            self._fps = inst if self._fps == 0 else 0.9 * self._fps + 0.1 * inst
        self._t_last = now

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_optical"

        # ---------------- 任务 1：足球检测 ----------------
        bres = self.ball_model.predict(frame, imgsz=int(
            self.get_parameter("imgsz").value), conf=float(
            self.get_parameter("ball_conf").value), verbose=False)[0]
        best = None
        for bx in bres.boxes:
            c = float(bx.conf[0])
            if best is None or c > best[0]:
                x1, y1, x2, y2 = [float(v) for v in bx.xyxy[0]]
                best = (c, (x1 + x2) / 2, (y1 + y2) / 2, x1, y1, x2, y2)
        bm = BallDetection()
        bm.header = header
        bm.frame_seq = self.seq
        bm.fps = float(self._fps)
        if best:
            c, u, v, x1, y1, x2, y2 = best
            bm.detected = True
            bm.u, bm.v, bm.confidence = float(u), float(v), float(c)
            bm.bbox_l, bm.bbox_t, bm.bbox_r, bm.bbox_b = (
                float(x1), float(y1), float(x2), float(y2))
        else:
            bm.detected = False
        self.pub_ball.publish(bm)

        # ---------------- 任务 2：标志定位 ----------------
        if self.seq % self.stride == 0:
            boxes = self._detect_boxes(
                self.marker_model, frame,
                float(self.get_parameter("marker_conf").value),
                int(self.get_parameter("imgsz").value),
                str(self.get_parameter("enhance").value))
            self.marker_count = len(boxes)
            if boxes:
                obs = np.array([[b[1], b[2]] for b in boxes], float)
                types = [b[3] for b in boxes]
                g, okm = self._pixels_to_ground(
                    obs, self.K, self.dist, self.R_cb, self.h)
                if okm.sum() >= 2:
                    sols = self._solve_pose_fixed(
                        g[okm], self.map_pts,
                        [types[i] for i in np.where(okm)[0]],
                        self.map_names)
                else:
                    sols = []
                if sols:
                    s = sols[0]      # 内点最多、残差最小的解
                    self.hist.append((s["x"], s["y"], s["yaw"]))
                    self.rms = float(s["rms"])
                else:
                    self.rms = None

        if self.marker_count == 0:
            self.valid, self.status = False, "INVALID: no landmark detected"
        elif self.rms is None:
            self.valid, self.status = False, "INVALID: no consistent solution"
        elif self.rms <= self.residual_max:
            self.valid, self.status = True, "OK"
        else:
            self.valid, self.status = False, "SEARCHING: high residual"

        pm = FieldPose()
        pm.header = header
        pm.header.frame_id = "map"
        pm.valid = self.valid
        if self.hist:
            arr = np.array(self.hist)
            pm.x_camera_map = float(np.median(arr[:, 0]))
            pm.y_camera_map = float(np.median(arr[:, 1]))
            pm.yaw_camera_map = float(np.median(arr[:, 2]))
        else:
            pm.x_camera_map = pm.y_camera_map = pm.yaw_camera_map = 0.0
        pm.marker_count = int(self.marker_count)
        pm.residual = float(self.rms) if self.rms is not None else 0.0
        pm.status = self.status
        pm.frame_seq = self.seq
        pm.fps = float(self._fps)
        self.pub_pose.publish(pm)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
