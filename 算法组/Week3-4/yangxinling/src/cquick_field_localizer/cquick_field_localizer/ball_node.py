#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ball_node.py —— 任务1 的 ROS2 发布节点（足球检测）

订阅：
    /image_raw                sensor_msgs/Image        （默认，可在参数里改）
发布：
    /soccer/ball              cquick_soccer_interfaces/TargetPosition
    /soccer/ball_boxes        cquick_soccer_interfaces/BallBox[]     （可选）
    /soccer/debug_image       sensor_msgs/Image                       （可选）

设计要点：
    1. 检测逻辑复用 Detector，和离线脚本完全一致，不写第二套；
    2. 图像编码兼容 bgr8/rgb8/mono8（很多相机驱动会发 rgb8，直接用 bgr8 会反色）；
    3. 发布"时间戳 + 帧号 + FPS"，任务书要求实时输出里必须有这些；
    4. 球没检测到时也发消息（detected=false），订阅端不用等超时。

运行：
    ros2 run cquick_field_localizer ball_node --ros-args \
        -p config:=config/camera.yaml -p show:=true

录屏演示时建议加 -p out_video:=out/ball_demo.mp4 把可视化也存下来。
"""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from .detector import Detector
from .pipeline import find_config, load_yaml
from .rosimg import bgr_to_imgmsg, imgmsg_to_bgr


class BallNode(Node):
    def __init__(self):
        super().__init__("ball_node")
        self.declare_parameter("config", "config/camera.yaml")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("show", False)
        self.declare_parameter("out_video", "")

        cfg_path = find_config(self.get_parameter("config").value)
        cfg = load_yaml(cfg_path)
        repo_root = os.path.dirname(os.path.dirname(cfg_path))
        self.detector = Detector(cfg.get("detector", {}), role="ball", base_dir=repo_root)
        self.cfg = cfg
        self.conf_thr = float((cfg.get("detector", {}) or {}).get("conf", 0.25))

        self.show = bool(self.get_parameter("show").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.frame_id = 0
        self.fps = 0.0
        self._t_prev = None
        self.writer = None
        out_video = self.get_parameter("out_video").value
        if out_video:
            os.makedirs(os.path.dirname(os.path.abspath(out_video)), exist_ok=True)
            self.writer = cv2.VideoWriter(
                out_video, cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (1280, 720))

        # 延迟导入自定义消息，避免没装接口包时 import 就崩
        try:
            from cquick_soccer_interfaces.msg import BallBox, TargetPosition
        except Exception as exc:
            self.get_logger().error(
                "找不到 cquick_soccer_interfaces，请先 colcon build 并 source install/setup.bash。"
                " 原始错误：%s" % exc)
            raise
        self.TargetPosition = TargetPosition
        self.BallBox = BallBox

        topic = self.get_parameter("image_topic").value
        self.pub = self.create_publisher(TargetPosition, "/soccer/ball", 10)
        self.pub_boxes = self.create_publisher(BallBox, "/soccer/ball_boxes", 10)
        self.pub_dbg = self.create_publisher(Image, "/soccer/debug_image", 2)
        self.sub = self.create_subscription(Image, topic, self.on_image, 2)

        for w in self.detector.warnings:
            self.get_logger().warn(w)
        self.get_logger().info("ball_node 启动：订阅 %s，检测器 backend=%s" % (topic, self.detector.backend))

    # ------------------------------------------------------------------
    def on_image(self, msg):
        if not rclpy.ok():
            return
        try:
            frame = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn("图像转换失败：%s" % exc)
            return

        t0 = time.time()
        dets = [d for d in self.detector.detect(frame) if d.cls_name == "Ball"]
        self.frame_id += 1
        dt = (t0 - self._t_prev) if self._t_prev else 0.0
        self._t_prev = t0
        if dt > 1e-6:
            self.fps = 0.9 * self.fps + 0.1 / dt if self.fps else 1.0 / dt

        stamp_txt = time.strftime("%Y-%m-%d %H:%M:%S")
        out = self.TargetPosition()
        out.stamp = stamp_txt
        out.frame_id = self.frame_id
        out.fps = float(self.fps)
        out.class_name = "ball"
        if dets:
            best = max(dets, key=lambda d: d.conf)
            u, v = best.center
            x1, y1, x2, y2 = [float(t) for t in best.bbox]
            out.detected = True
            out.confidence = float(best.conf)
            out.x1, out.y1, out.x2, out.y2 = x1, y1, x2, y2
            out.center_u = float(u)
            out.center_v = float(v)
            out.area = float(best.area())
        else:
            out.detected = False
            out.confidence = 0.0
            out.x1 = out.y1 = out.x2 = out.y2 = 0.0
            out.center_u = out.center_v = -1.0
            out.area = 0.0
        try:
            self.pub.publish(out)
        except Exception:
            return

        boxes = self.BallBox()
        for d in dets:
            boxes.confidence = float(d.conf)
            boxes.bbox = [float(x) for x in d.bbox]
            boxes.class_id = int(d.cls_id)
            boxes.class_name = d.cls_name
            try:
                self.pub_boxes.publish(boxes)
            except Exception:
                break

        if self.publish_debug or self.show or self.writer is not None:
            vis = self._draw(frame, dets, out)
            if self.publish_debug:
                try:
                    self.pub_dbg.publish(bgr_to_imgmsg(
                        vis, stamp=msg.header.stamp, frame_id=msg.header.frame_id or "camera"))
                except Exception:
                    pass
            if self.writer is not None:
                self.writer.write(vis)
            if self.show:
                cv2.imshow("ball_node", vis)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    rclpy.shutdown()

    # ------------------------------------------------------------------
    def _draw(self, frame, dets, out):
        vis = frame.copy()
        h, w = vis.shape[:2]
        for d in dets:
            x1, y1, x2, y2 = [int(round(v)) for v in d.bbox]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (60, 220, 60), 2)
            u, v = d.center
            cv2.circle(vis, (int(u), int(v)), 4, (0, 0, 255), -1)
            cv2.putText(vis, "ball %.2f (%d,%d)" % (d.conf, int(u), int(v)),
                        (x1, max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 220, 60), 2)
        cv2.rectangle(vis, (0, 0), (w, 28), (20, 20, 20), -1)
        cv2.putText(vis, "frame %d | %s | FPS %.1f | detected=%s" %
                    (out.frame_id, out.stamp, self.fps, out.detected),
                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
        return vis

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        if self.show:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = BallNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # 同 localizer_node：Ctrl-C 时 rclpy 抛 ExternalShutdownException，接住以免刷 traceback。
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
