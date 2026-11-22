#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localizer_node.py —— 任务2 的 ROS2 节点（单目场地自定位）

订阅：
    /image_raw             sensor_msgs/Image
    /camera_info           sensor_msgs/CameraInfo（可选；收到就用它的内参覆盖 yaml）
发布：
    /localization/field_pose   cquick_soccer_interfaces/FieldPose
    /localization/debug_image  sensor_msgs/Image（左右分屏画面，可选）

参数：
    config        配置文件（默认 config/camera.yaml）
    image_topic   图像话题
    show          是否弹窗（现场演示/录屏时打开）
    out_video     把分屏画面录成 mp4（录 B 站演示视频最省事的方式）
    publish_debug 是否发布 debug_image

为什么订阅 CameraInfo 很有用：
    现场换相机/改分辨率时，yaml 里的内参就失效了。只要驱动发布了
    /camera_info，节点会自动用它，避免"忘了改 yaml 结果定位全错"。
"""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .field_view import FieldView, make_split_view
from .localize import draw_left_panel
from .pipeline import LocalizerPipeline, find_config
from .rosimg import bgr_to_imgmsg, imgmsg_to_bgr


class LocalizerNode(Node):
    def __init__(self):
        super().__init__("field_localizer")
        self.declare_parameter("config", "config/camera.yaml")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("show", False)
        self.declare_parameter("out_video", "")
        self.declare_parameter("use_camera_info", True)
        # 初始位姿先验：场地镜像对称，不给先验时首帧必然 ambiguous
        self.declare_parameter("prior_x", 0.0)
        self.declare_parameter("prior_y", 0.0)
        self.declare_parameter("prior_yaw_deg", 0.0)
        self.declare_parameter("use_prior", False)

        cfg_path = find_config(self.get_parameter("config").value)
        self.pipe = LocalizerPipeline(cfg_path)
        self.get_logger().info(self.pipe.describe())
        if bool(self.get_parameter("use_prior").value):
            px = float(self.get_parameter("prior_x").value)
            py = float(self.get_parameter("prior_y").value)
            pyaw = float(self.get_parameter("prior_yaw_deg").value)
            self.pipe.set_initial_prior(px, py, pyaw)
            self.get_logger().info(
                "[先验] x=%.2f m y=%.2f m yaw=%.1f deg（用于破除镜像歧义）"
                % (px, py, pyaw))

        vcfg = self.pipe.cfg.get("viewer", {}) or {}
        self.total_w = int(vcfg.get("width", 1280))
        self.total_h = int(vcfg.get("height", 720))
        self.fview = FieldView(self.pipe.field, self.pipe.geometry,
                               width=self.total_w // 2, height=self.total_h,
                               pad_m=float(vcfg.get("field_pad_m", 0.6)))
        self.show = bool(self.get_parameter("show").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.use_camera_info = bool(self.get_parameter("use_camera_info").value)
        self.frame_id = 0
        self.update_hz = 0.0
        self._t_valid = None
        self.writer = None
        out_video = self.get_parameter("out_video").value
        if out_video:
            os.makedirs(os.path.dirname(os.path.abspath(out_video)), exist_ok=True)
            self.writer = cv2.VideoWriter(out_video, cv2.VideoWriter_fourcc(*"mp4v"),
                                          30.0, (self.total_w, self.total_h))

        try:
            from cquick_soccer_interfaces.msg import FieldPose
        except Exception as exc:
            self.get_logger().error(
                "找不到 cquick_soccer_interfaces，请先 colcon build 并 source install/setup.bash。"
                " 原始错误：%s" % exc)
            raise
        self.FieldPose = FieldPose
        self.pub_pose = self.create_publisher(FieldPose, "/localization/field_pose", 10)
        self.pub_dbg = self.create_publisher(Image, "/localization/debug_image", 2)
        self.sub = self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.on_image, 2)
        if self.use_camera_info:
            self.create_subscription(CameraInfo, "/camera_info", self.on_camera_info, 2)

        self.get_logger().info("field_localizer 启动：话题 %s，输出 /localization/field_pose"
                               % self.get_parameter("image_topic").value)

    # ------------------------------------------------------------------
    def on_camera_info(self, msg):
        """用驱动的 CameraInfo 覆盖 yaml 内参（K 是 3x3 行优先，D 是畸变）。"""
        try:
            K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            if abs(K[0, 0]) < 1e-6:
                return
            self.pipe.camera.K = K
            self.pipe.camera.K_inv = np.linalg.inv(K)
            if len(msg.d) >= 4:
                self.pipe.camera.D = np.array(msg.d, dtype=np.float64).ravel()[:5]
            self.use_camera_info = False        # 只用第一帧，避免每帧都刷
            self.get_logger().info("已用 /camera_info 覆盖内参：fx=%.2f fy=%.2f cx=%.2f cy=%.2f"
                                   % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
        except Exception as exc:
            self.get_logger().warn("CameraInfo 解析失败：%s" % exc)

    # ------------------------------------------------------------------
    def on_image(self, msg):
        # Ctrl-C 后 rclpy 已 shutdown，队列里残留的消息会再进来一次；
        # 此时任何 publish 都会抛 RCLError，直接跳过。
        if not rclpy.ok():
            return
        try:
            frame = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn("图像转换失败：%s" % exc)
            return

        self.frame_id += 1
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        rec = self.pipe.process(frame, self.frame_id, stamp)
        pose = rec["pose"]

        out = self.FieldPose()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.valid = bool(pose.get("valid"))
        # 本实现把相机光心定义为载体参考原点（相机直接固定在支架上，
        # 支架参考点就是相机光心的地面投影），所以两者数值相同；
        # 但字段仍然分别填写，符合细则"不得把底座坐标未换算就标成相机光心"的要求。
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw = float(pose.get("yaw_deg", 0.0))
        out.x_camera_map = x
        out.y_camera_map = y
        out.yaw_camera_map = yaw
        out.x_carrier_map = x
        out.y_carrier_map = y
        out.yaw_carrier_map = yaw
        out.marker_count = int(pose.get("marker_count", 0))
        res = pose.get("residual", float("nan"))
        out.residual = float(res) if np.isfinite(res) else -1.0
        out.status = str(pose.get("status", "lost"))
        out.fps = float(rec["fps"])
        out.processing_ms = float(rec.get("processing_ms", 0.0))
        # 位姿更新频率：只对"有效位姿"计时，用指数滑动平均，避免抖动
        now = time.time()
        if out.valid:
            if self._t_valid is not None:
                dt = max(1e-3, now - self._t_valid)
                hz = 1.0 / dt
                self.update_hz = hz if self.update_hz <= 0 else 0.9 * self.update_hz + 0.1 * hz
            self._t_valid = now
        out.update_hz = float(self.update_hz)
        alt = pose.get("second")
        out.has_alternative = alt is not None
        if alt:
            out.alt_x = float(alt["x"])
            out.alt_y = float(alt["y"])
            out.alt_yaw_deg = float(alt["yaw_deg"])
        try:
            self.pub_pose.publish(out)
        except Exception:
            return

        if self.publish_debug or self.show or self.writer is not None:
            left = draw_left_panel(frame, rec, width=self.total_w // 2, height=self.total_h)
            pose_view = dict(pose)
            pose_view.update(update_hz=out.update_hz, processing_ms=out.processing_ms,
                             status=out.status)
            right = self.fview.render(pose=pose_view, observations=rec["obs_map"],
                                      status=out.status, fps=rec["fps"],
                                      frame_id=self.frame_id, stamp=stamp)
            vis = make_split_view(left, right, self.total_w, self.total_h)
            if self.publish_debug:
                try:
                    self.pub_dbg.publish(bgr_to_imgmsg(
                        vis, stamp=self.get_clock().now().to_msg(), frame_id="map"))
                except Exception:
                    pass
            if self.writer is not None:
                self.writer.write(vis)
            if self.show:
                cv2.imshow("field_localizer", vis)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    rclpy.shutdown()

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        if self.show:
            cv2.destroyAllWindows()
        s = self.pipe.summary()
        # 关节点时 rosout 的 context 已经失效，再走 get_logger().info() 会让 rcl 打
        # "Failed to publish log message to rosout"，所以这段统计一律打到 stdout。
        print("[cquick] frames=%d fps_mean=%.1f valid_ratio=%.1f%% status=%s"
              % (s["frames"], s["fps_mean"], 100 * s["valid_ratio"], s["status_counts"]),
              flush=True)
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = LocalizerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 时 rclpy 会抛 ExternalShutdownException；不接住就会打一大段 traceback。
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
