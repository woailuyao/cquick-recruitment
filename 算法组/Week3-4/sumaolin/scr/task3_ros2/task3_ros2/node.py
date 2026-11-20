#!/usr/bin/env python3
"""Task3 的 ROS2 封装节点。

这个文件不复制 task3.py、marker.pt、ball.pt，而是通过 task3_root 参数找到
已有的 scr/task3.py，复用其中的标定、PnP、匹配和绘制函数。

话题：/task3/localization
消息：task3_interfaces/msg/Localization
"""

import importlib.util
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from task3_interfaces.msg import BallDetection, Localization, MarkerDetection

TOPIC = "/task3/localization"


def load_task3(root: Path):
    """从源码目录按文件路径加载 task3.py，不复制、不安装。"""
    source = root / "task3.py"
    if not source.is_file():
        raise RuntimeError(
            f"找不到 {source}。请在 scr 目录下运行，或传入 "
            f"-p task3_root:=/absolute/path/to/scr"
        )
    spec = importlib.util.spec_from_file_location("task3_core", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Task3Node(Node):
    def __init__(self):
        super().__init__("task3_node")

        # task3_root 默认取当前工作目录；建议在 scr/ 下 ros2 run，
        # 或显式传 -p task3_root:=/path/to/scr
        self.declare_parameter("task3_root", os.environ.get("TASK3_ROOT", str(Path.cwd())))
        self.declare_parameter("source", "auto")
        self.declare_parameter("camera_backend", "auto")
        self.declare_parameter("marker_model", "")
        self.declare_parameter("ball_model", "")
        self.declare_parameter("field", "")
        self.declare_parameter("calib", "")
        self.declare_parameter("marker_conf", 0.2)
        self.declare_parameter("ball_conf", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("pose_mode", "pnp")
        self.declare_parameter("display", False)

        root = Path(self.get_parameter("task3_root").value).expanduser().resolve()
        if not (root / "task3.py").is_file():
            for candidate in (Path.cwd(), Path(__file__).resolve().parents[2]):
                if (candidate / "task3.py").is_file():
                    root = candidate.resolve()
                    break
        self.task3 = load_task3(root)

        marker_model = self.get_parameter("marker_model").value or str(root / "marker.pt")
        ball_model = self.get_parameter("ball_model").value or str(root / "ball.pt")
        field_path = self.get_parameter("field").value or str(root.parent / "config" / "field.yaml")
        calib_path = self.get_parameter("calib").value
        if not calib_path:
            candidate = root.parent / "config" / "camera_intrinsics.yaml"
            calib_path = str(candidate) if candidate.is_file() else ""

        self.pose_mode = str(self.get_parameter("pose_mode").value)
        self.marker_conf = float(self.get_parameter("marker_conf").value)
        self.ball_conf = float(self.get_parameter("ball_conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.display = bool(self.get_parameter("display").value)

        cam_args = SimpleNamespace(
            calib=calib_path,
            mount_height=None,
            pitch=None,
            roll=None,
            yaw=None,
        )
        self.cam = self.task3.build_camera(
            cam_args, require_mount=(self.pose_mode == "plane")
        )
        self.field = self.task3.build_field_model(self.task3.load_field(field_path))
        self.r_bc = self.task3.carrier_from_camera(
            self.cam["pitch_deg"], self.cam["roll_deg"], self.cam["yaw_deg"]
        )
        self.origin = np.array(
            [self.cam["mount_x"], self.cam["mount_y"], self.cam["mount_height"]],
            dtype=np.float64,
        )
        self.max_range = float(self.cam["max_range"])

        try:
            from ultralytics import YOLO
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "缺少 ultralytics。请在运行 ros2 的同一个 Python 环境里安装：\n"
                "  python3 -m pip install --user ultralytics\n"
                "然后确认：python3 -c \"import ultralytics\"\n"
                "如果提示 externally-managed-environment，改用 venv 或加 --break-system-packages。"
            ) from exc

        self.marker_model = YOLO(marker_model)
        self.ball_model = YOLO(ball_model)
        self.cap = self.task3.open_capture(
            self.get_parameter("source").value,
            self.get_parameter("camera_backend").value,
        )

        self.publisher = self.create_publisher(Localization, TOPIC, 10)
        self.fps = 0.0
        self.last_time = time.perf_counter()
        self.last_log = 0.0
        self.frame_idx = 0
        self.last_pose = None
        self.last_pose_frame = -10 ** 9
        self.timer = self.create_timer(0.03, self.tick)

        self.get_logger().info(
            f"task3 node started: topic={TOPIC} source={self.get_parameter('source').value} "
            f"mode={self.pose_mode} root={root}"
        )

    def _marker_detections(self, result):
        boxes = result.boxes
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy().astype(int)
        names = result.names
        detections = []
        for i in range(len(xyxy)):
            name = (names.get(int(classes[i]), str(classes[i]))
                    if isinstance(names, dict) else names[int(classes[i])])
            short = self.task3.short_type(name)
            if short is None:
                continue
            x1, y1, x2, y2 = xyxy[i]
            detections.append({
                "short": short,
                "conf": float(confs[i]),
                "box": (x1, y1, x2, y2),
                "uv": (float((x1 + x2) * 0.5), float((y1 + y2) * 0.5)),
            })
        return detections

    def _localize(self, detections, k_matrix, dist):
        if self.pose_mode == "pnp":
            pnp_detections = [
                {"type": d["short"], "uv": d["uv"], "confidence": d["conf"]}
                for d in detections
            ]
            prev_pose = None
            if (self.last_pose is not None
                    and self.frame_idx - self.last_pose_frame <= self.task3.PNP_TEMPORAL_MAX_GAP):
                prev_pose = self.last_pose
            pose = self.task3.solve_pose_pnp(
                pnp_detections, self.field, k_matrix, dist, prev_pose=prev_pose)
            if pose["valid"]:
                self.last_pose = pose
                self.last_pose_frame = self.frame_idx
            display = [
                {"type": d["type"], "u": d["uv"][0], "v": d["uv"][1],
                 "confidence": d["confidence"]}
                for d in pnp_detections
            ]
            return pose, display

        observations = []
        if detections:
            uv = np.array([d["uv"] for d in detections], dtype=np.float64).reshape(-1, 1, 2)
            normalized = cv2.undistortPoints(uv, k_matrix, dist).reshape(-1, 2)
        else:
            normalized = np.empty((0, 2), dtype=np.float64)
        for detection, (nx, ny) in zip(detections, normalized):
            ground = self.task3.ray_to_ground(nx, ny, self.r_bc, self.origin, self.max_range)
            if ground is not None:
                observations.append({
                    "type": detection["short"],
                    "u": detection["uv"][0],
                    "v": detection["uv"][1],
                    "x": float(ground[0]),
                    "y": float(ground[1]),
                    "confidence": detection["conf"],
                })
        pose = self.task3.solve_pose(observations, self.field, self.cam)
        return pose, observations

    def _publish(self, pose, display_markers, balls):
        msg = Localization()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.valid = bool(pose["valid"])
        msg.temporal = bool(pose.get("temporal", False))
        msg.status = str(pose["status"])
        msg.pose_mode = self.pose_mode
        msg.marker_count = int(pose["marker_count"])
        msg.residual = 0.0 if pose["residual"] is None else float(pose["residual"])
        msg.residual_unit = str(pose.get("residual_unit", "m"))
        msg.x_camera_map = 0.0 if pose["x_camera_map"] is None else float(pose["x_camera_map"])
        msg.y_camera_map = 0.0 if pose["y_camera_map"] is None else float(pose["y_camera_map"])
        msg.yaw_camera_map = 0.0 if pose["yaw_camera_map"] is None else float(pose["yaw_camera_map"])
        msg.fps = float(self.fps)

        for item in display_markers:
            marker = MarkerDetection()
            marker.type = str(item["type"])
            marker.u = float(item["u"])
            marker.v = float(item["v"])
            marker.confidence = float(item["confidence"])
            marker.has_ground_xy = "x" in item and "y" in item
            marker.x = float(item["x"]) if marker.has_ground_xy else 0.0
            marker.y = float(item["y"]) if marker.has_ground_xy else 0.0
            msg.markers.append(marker)

        for ball in balls:
            item = BallDetection()
            item.confidence = float(ball["confidence"])
            item.center_u = int(ball["center_u"])
            item.center_v = int(ball["center_v"])
            item.x1, item.y1, item.x2, item.y2 = [float(v) for v in ball["bbox"]]
            msg.balls.append(item)

        self.publisher.publish(msg)
        return msg

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("读取视频帧失败，停止节点")
            self.timer.cancel()
            return
        self.frame_idx += 1

        h, w = frame.shape[:2]
        k_matrix, dist = self.task3.scaled_intrinsics(self.cam, w, h)

        t0 = time.perf_counter()
        marker_result = self.marker_model.predict(
            frame, conf=self.marker_conf, imgsz=self.imgsz,
            device=self.device or None, verbose=False,
        )[0]
        ball_result = self.ball_model.predict(
            frame, conf=self.ball_conf, imgsz=self.imgsz,
            device=self.device or None, verbose=False,
        )[0]
        infer_ms = (time.perf_counter() - t0) * 1000.0

        detections = self._marker_detections(marker_result)
        pose, display_markers = self._localize(detections, k_matrix, dist)

        boxes = ball_result.boxes
        b_xyxy = boxes.xyxy.cpu().numpy()
        b_conf = boxes.conf.cpu().numpy()
        balls = []
        for i in range(len(b_xyxy)):
            x1, y1, x2, y2 = b_xyxy[i]
            balls.append({
                "confidence": float(b_conf[i]),
                "center_u": int(round((x1 + x2) * 0.5)),
                "center_v": int(round((y1 + y2) * 0.5)),
                "bbox": [float(x1), float(y1), float(x2), float(y2)],
            })

        now = time.perf_counter()
        dt = now - self.last_time
        self.last_time = now
        if dt > 0:
            inst = 1.0 / dt
            self.fps = inst if self.fps == 0.0 else 0.9 * self.fps + 0.1 * inst

        msg = self._publish(pose, display_markers, balls)

        if now - self.last_log >= 1.0:
            self.get_logger().info(
                f"valid={msg.valid} status={msg.status} markers={msg.marker_count} "
                f"pose=({msg.x_camera_map:.2f}, {msg.y_camera_map:.2f}, {msg.yaw_camera_map:.1f}) "
                f"balls={len(balls)} infer={infer_ms:.0f}ms"
            )
            self.last_log = now

        if self.display:
            self._render(frame, pose, detections, display_markers, balls, infer_ms)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                self.timer.cancel()
                rclpy.shutdown()

    def _render(self, frame, pose, detections, display_markers, balls, infer_ms):
        by_uv = {(round(d["uv"][0]), round(d["uv"][1])): d for d in detections}
        for item in display_markers:
            key = (round(item["u"]), round(item["v"]))
            detection = by_uv.get(key)
            if detection is None:
                continue
            ground = (item["x"], item["y"]) if ("x" in item and "y" in item) else None
            coord_text = None if ground is not None else f"({item['u']:.0f}, {item['v']:.0f})"
            self.task3.draw_marker(
                frame, detection["box"], detection["short"], detection["conf"],
                int(round(item["u"])), int(round(item["v"])), ground,
                coord_text=coord_text,
            )
        for ball in balls:
            self.task3.draw_ball(frame, ball["bbox"], ball["confidence"])

        left = self.task3.fit_panel(frame, self.task3.LEFT_W, self.task3.LEFT_H)
        map_panel = self.task3.draw_map(self.field, pose, self.fps, infer_ms)
        canvas = np.hstack([left, map_panel])
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.task3.put_text(canvas, self.task3.OPERATOR_NAME, (10, 30), 0.9, (0, 255, 255), 2)
        tw = cv2.getTextSize(stamp, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0][0]
        self.task3.put_text(canvas, stamp, (canvas.shape[1] - tw - 10, 30), 0.6, (0, 255, 255), 1)
        cv2.setWindowTitle("task3_ros2", f"task3_ros2 - {self.task3.OPERATOR_NAME}")
        cv2.imshow("task3_ros2", canvas)

    def destroy_node(self):
        try:
            if getattr(self, "cap", None) is not None:
                self.cap.release()
        finally:
            cv2.destroyAllWindows()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Task3Node()
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
