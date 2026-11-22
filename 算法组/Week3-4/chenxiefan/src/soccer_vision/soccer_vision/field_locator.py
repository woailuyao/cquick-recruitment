#!/usr/bin/env python3

import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from soccer_interfaces.msg import FieldPose

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import locate_field as lf

class FieldLocator(Node):
    def __init__(self):
        super().__init__("field_locator")
        self.declare_parameter("source", "http://192.168.56.1:8080/video")
        self.declare_parameter("model", os.path.join(ROOT, "models", "marker_best.pt"))
        self.declare_parameter("field", os.path.join(ROOT, "config", "field.yaml"))
        self.declare_parameter("camera", os.path.join(ROOT, "config", "camera.yaml"))
        self.declare_parameter("extrinsics", os.path.join(ROOT, "config", "camera_extrinsics.yaml"))
        self.declare_parameter("conf", 0.15)
        self.declare_parameter("imgsz", 960)

        self.source = str(self.get_parameter("source").value)
        self.model_path = str(self.get_parameter("model").value)
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)

        cam = lf.load_yaml(str(self.get_parameter("camera").value)) or {}
        extr = lf.load_yaml(str(self.get_parameter("extrinsics").value)) or {}
        field = lf.load_yaml(str(self.get_parameter("field").value)) or {}
        self.markers = lf.load_field_markers(field)
        self.K = np.array(cam["camera_matrix"], dtype=float)
        self.dist = np.array(cam["dist_coeffs"], dtype=float)
        self.R = np.array(extr["R_base_camera"], dtype=float)
        self.t = np.array(extr["t_base_camera"], dtype=float)

        self.pub = self.create_publisher(FieldPose, "/localization/field_pose", 10)
        self.model = None
        self._warned = False
        self.tracker = lf.MarkerTracker()
        self.anchor = None

        if self.source.startswith("http"):
            self.cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
        elif self.source.startswith("camera:"):
            self.cap = cv2.VideoCapture(int(self.source.split(":", 1)[1] or 0), cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        else:
            self.cap = cv2.VideoCapture(self.source)

        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info(
            f"field_locator: source={self.source}, model={self.model_path}, "
            f"markers={len(self.markers)}, imgsz={self.imgsz}")

    def _load_model(self):
        if self.model is not None:
            return True
        if not os.path.exists(self.model_path):
            if not self._warned:
                self.get_logger().warn(f"模型不存在：{self.model_path}")
                self._warned = True
            return False
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().warn("未安装 ultralytics")
            return False
        self.model = YOLO(self.model_path)
        return True

    def _estimate(self, frame):
        if not self._load_model():
            return {"valid": False, "status": "no_model", "x": 0.0, "y": 0.0,
                    "yaw_deg": 0.0, "marker_count": 0, "residual": 0.0}

        results = self.model.predict(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)
        raw = []
        for box in results[0].boxes:
            name = results[0].names.get(int(box.cls[0]), "")
            if name not in lf.CLASSES:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx0, cy0 = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            prior = None
            for trc in self.tracker.tracks:
                if trc["type"] != name:
                    continue
                tx1, ty1, tx2, ty2 = trc["box"]
                tcx, tcy = (tx1 + tx2) / 2.0, (ty1 + ty2) / 2.0
                if abs(tcx - cx0) < 110 and abs(tcy - cy0) < 110:
                    prior = (trc["u"] + (cx0 - tcx), trc["v"] + (cy0 - tcy))
                    break
            u, v = lf.refine_landmark(frame, name, (x1, y1, x2, y2), prior=prior)
            raw.append({"type": name, "conf": float(box.conf[0]), "u": u, "v": v,
                        "box": (x1, y1, x2, y2)})

        raw = lf.dedupe_detections(raw)
        obs = []
        for tr in self.tracker.update(raw):
            ground = lf.pixel_to_ground(tr["u"], tr["v"], self.K, self.dist, self.R, self.t)
            if ground is None:
                continue
            obs.append({"type": tr["type"], "x": ground[0], "y": ground[1],
                        "conf": tr["conf"]})

        obs = lf.add_virtual_center(obs)
        pose = lf.solve_pose(obs, self.markers, prior=self.anchor)
        if pose.get("valid"):
            if self.anchor is None:
                if pose.get("marker_count", 0) >= 4:
                    self.anchor = (pose["x"], pose["y"], pose["yaw_deg"])
            else:
                ax, ay, ayaw = self.anchor
                dpos = ((pose["x"] - ax) ** 2 + (pose["y"] - ay) ** 2) ** 0.5
                dyaw = abs((pose["yaw_deg"] - ayaw + 180) % 360 - 180)
                if dpos > 0.15 or dyaw > 5.0:
                    self.anchor = (pose["x"], pose["y"], pose["yaw_deg"])
        return pose

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return
        est = self._estimate(frame)
        msg = FieldPose()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.valid = bool(est["valid"])
        msg.x = float(est["x"])
        msg.y = float(est["y"])
        msg.yaw_deg = float(est["yaw_deg"])
        msg.marker_count = int(est["marker_count"])
        msg.residual = float(est["residual"])
        msg.status = str(est["status"])
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = FieldLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
