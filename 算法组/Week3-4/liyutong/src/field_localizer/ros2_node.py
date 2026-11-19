#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 节点：足球检测 + 单目场地自定位

⚠️ 说明：本文件在 Windows 开发机上**没有实际运行过**（该机未安装 ROS2）。
    纯 Python 的算法链路（localize.py / demo.py）已在 Windows 上跑通并做过
    合成自测；本文件只是把同一条链路包成 ROS2 节点，需要在实验室 Ubuntu
    机器上 `colcon`/`ros2 run` 实测一次。README 里已如实标注这一点。

话题
    订阅  /camera/image_raw                  sensor_msgs/Image          单目 RGB
    发布  /perception/ball                   std_msgs/Float32MultiArray 足球检测
          /perception/field_pose             geometry_msgs/PoseWithCovarianceStamped
          /perception/field_status           std_msgs/Float32MultiArray 定位质量
          /perception/debug_image            sensor_msgs/Image          可视化（可选）

消息字段约定
    /perception/ball           每 5 个数一组：[cx, cy, w, h, conf]，cx/cy 为像素中心，
                               w/h 为像素宽高。像素坐标原点在左上角，x 向右、y 向下。
    /perception/field_pose     header.frame_id = "map"（field.yaml 坐标系）。
                               pose.position.x  = x_camera_map  (m)
                               pose.position.y  = y_camera_map  (m)
                               pose.position.z  = 相机高度 h     (m)
                               pose.orientation = 绕 Z 轴 yaw_camera_map 的四元数，
                                                 0 表示光轴水平投影指向 +X（球门方向）
                               协方差只用对角线：x/y/yaw 的方差
    /perception/field_status   [marker_count, residual, valid, status_code]
                               residual 单位像素；valid 1=有效 0=无效；
                               status_code: 0=OK 1=NO_MARKERS 2=NO_MATCH
                                            3=TOO_FEW_INLIERS 4=HIGH_RESIDUAL
                                            5=BELOW_HORIZON 6=REPROJ_FAIL
                                            7=JUMP_REJECTED

运行
    # 先确认 localize.py / field.yaml / 标定文件在同目录或已加入 PYTHONPATH
    python3 ros2_node.py --ros-args -p ball_model:=best_ball.pt -p marker_model:=best_marker.pt \
        -p camera_height:=0.45 -p image_topic:=/camera/image_raw
"""
import math
import os
import sys

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import localize as L      # noqa: E402

STATUS_CODE = {
    "OK": 0, "NO_MARKERS": 1, "NO_MATCH": 2, "TOO_FEW_INLIERS": 3,
    "HIGH_RESIDUAL": 4, "BELOW_HORIZON": 5, "REPROJ_FAIL": 6,
    "JUMP_REJECTED": 7,
}


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("field_localizer")
        p = self.declare_parameter
        self.ball_model_path = p("ball_model", "best_ball.pt").value
        self.marker_model_path = p("marker_model", "best_marker.pt").value
        self.image_topic = p("image_topic", "/camera/image_raw").value
        self.conf_ball = float(p("conf_ball", 0.40).value)
        self.conf_mark = float(p("conf_mark", 0.35).value)
        self.publish_debug = bool(p("publish_debug_image", True).value)

        self.K, self.dist = L.load_intrinsics()
        self.R_gc, h_yaml = L.load_extrinsics()
        self.markers, self.goal, self.fld = L.load_field()
        self.height = float(p("camera_height", h_yaml).value)
        self.field_dims = (float(self.fld["length_m"]), float(self.fld["width_m"]))
        self.prev = None

        from ultralytics import YOLO
        self.ball = YOLO(self.ball_model_path) if os.path.exists(self.ball_model_path) else None
        self.mk = YOLO(self.marker_model_path) if os.path.exists(self.marker_model_path) else None
        if self.ball is None:
            self.get_logger().warn(f"找不到足球模型 {self.ball_model_path}，只发布定位")
        if self.mk is None:
            self.get_logger().warn(f"找不到标志模型 {self.marker_model_path}，定位不会出解")

        self.bridge = CvBridge() if CvBridge else None
        if self.bridge is None:
            self.get_logger().error("没有 cv_bridge：sudo apt install ros-<distro>-cv-bridge")

        self.pub_ball = self.create_publisher(Float32MultiArray, "/perception/ball", 10)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped,
                                              "/perception/field_pose", 10)
        self.pub_stat = self.create_publisher(Float32MultiArray, "/perception/field_status", 10)
        self.pub_img = (self.create_publisher(Image, "/perception/debug_image", 1)
                        if self.publish_debug else None)
        self.create_subscription(Image, self.image_topic, self.on_image, 1)

        self.get_logger().info(
            f"订阅 {self.image_topic}｜高度 {self.height:.3f} m｜"
            f"标志点 {len(self.markers)} 个｜球模型 {'有' if self.ball else '无'}｜"
            f"标志模型 {'有' if self.mk else '无'}")

    # ---------------- 回调 ----------------
    def on_image(self, msg):
        if self.bridge is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        H, W = frame.shape[:2]

        # 任务 1
        ball_arr = []
        if self.ball is not None:
            r = self.ball.predict(frame, conf=self.conf_ball, imgsz=640, verbose=False)[0]
            for b in r.boxes:
                x1, y1, x2, y2 = map(float, b.xyxy[0])
                ball_arr += [0.5 * (x1 + x2), 0.5 * (y1 + y2),
                             x2 - x1, y2 - y1, float(b.conf[0])]
        m = Float32MultiArray()
        m.data = ball_arr
        self.pub_ball.publish(m)

        # 任务 2
        dets = []
        if self.mk is not None:
            r2 = self.mk.predict(frame, conf=self.conf_mark, imgsz=640, verbose=False)[0]
            for b in r2.boxes:
                x1, y1, x2, y2 = map(float, b.xyxy[0])
                dets.append((self.mk.names[int(b.cls[0])],
                             0.5 * (x1 + x2), 0.5 * (y1 + y2)))

        res = L.localize(dets, self.K, self.dist, self.R_gc, self.height, self.markers,
                         goal=self.goal, require_margin=(H, W),
                         field_dims=self.field_dims, prev=self.prev)
        if res["valid"]:
            self.prev = dict(x=res["x_camera_map"], y=res["y_camera_map"],
                             yaw=res["yaw_camera_map"])
        elif res.get("marker_count", 0) < 2:
            self.prev = None

        st = Float32MultiArray()
        st.data = [float(res.get("marker_count") or 0),
                   float(res.get("residual") or -1.0),
                   1.0 if res["valid"] else 0.0,
                   float(STATUS_CODE.get(res.get("status", ""), -1))]
        self.pub_stat.publish(st)

        pose = PoseWithCovarianceStamped()
        pose.header = msg.header
        pose.header.frame_id = "map"
        if res["valid"]:
            yaw = math.radians(res["yaw_camera_map"])
            pose.pose.pose.position.x = float(res["x_camera_map"])
            pose.pose.pose.position.y = float(res["y_camera_map"])
            pose.pose.pose.position.z = float(self.height)
            pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
            # 只有 x / y / yaw 是真观测，其余给大方差让上层知道别用
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.04          # 0.2 m
            cov[35] = math.radians(5.0) ** 2
            cov[14] = cov[21] = cov[28] = 1e3
            pose.pose.covariance = cov
        else:
            pose.pose.pose.position.x = float("nan")
            pose.pose.pose.position.y = float("nan")
            pose.pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

        if self.pub_img is not None:
            vis = frame.copy()
            for t, u, v in dets:
                cv2.circle(vis, (int(u), int(v)), 3, (255, 0, 255), -1)
            if res["valid"]:
                cv2.putText(vis, f"x={res['x_camera_map']:+.2f} y={res['y_camera_map']:+.2f}"
                                 f" yaw={res['yaw_camera_map']:+.1f}", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(vis, encoding="bgr8"))


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
