"""ROS2 节点 field_pose_node：/soccer/image_raw → 标志检测 + 地图匹配 → /localization/field_pose。

定位链路：四类标志检测 → 亚像素交点细化（task2/refine.py）→
类别级 RANSAC 多候选匹配（task2/correspondence.py）→ PnP + 高度先验消歧 →
输出 C0 地图下的 camera_frame 位姿与 marker_count / residual / valid / status。

红线：marker 模型缺失或标志不足时**必须发布 valid=false**，
绝不填假坐标（细则「定位失败仍伪造有效位姿 −10」）。

⚠️ 开发机是 Windows 11（无 ROS2 Humble），本文件未在实机跑通，
   20 号下午到实验室 Ubuntu 机按 src/README.md 验证。

查话题：ros2 topic echo /localization/field_pose
"""
import sys
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_interfaces.msg import FieldPose

WEEK3 = Path(__file__).resolve().parents[3]
if str(WEEK3) not in sys.path:
    sys.path.insert(0, str(WEEK3))
from task2 import FieldLocalizer  # noqa: E402


class FieldPoseNode(Node):
    def __init__(self):
        super().__init__("field_pose_node")
        self.declare_parameter("marker_model", "")      # 空 = 未训练，valid 恒 false
        self.declare_parameter("intrinsics", "config/camera_intrinsics_fieldvideo.yaml")
        self.declare_parameter("field", "config/field.yaml")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("h_prior", 0.55)
        self.declare_parameter("residual_thresh", 15.0)
        self.declare_parameter("pose_every", 5)         # 每 N 帧解一次位姿

        marker_path = str(self.get_parameter("marker_model").value)
        intrinsics = self._abs(self.get_parameter("intrinsics").value)
        field_yaml = self._abs(self.get_parameter("field").value)
        self.loc = FieldLocalizer(intrinsics, field_yaml)
        self.conf = float(self.get_parameter("conf").value)
        self.h_prior = float(self.get_parameter("h_prior").value)
        self.residual_thresh = float(self.get_parameter("residual_thresh").value)
        self.pose_every = max(1, int(self.get_parameter("pose_every").value))

        self.marker_model = None
        if marker_path:
            from ultralytics import YOLO
            mp = Path(marker_path)
            if not mp.is_absolute():
                mp = WEEK3 / mp
            if not mp.exists():
                raise RuntimeError(f"marker 模型不存在：{mp}")
            self.marker_model = YOLO(str(mp))
            self.get_logger().info(f"已载入 marker 模型 {mp.name}")
        else:
            self.get_logger().warn(
                "未给 marker_model：将只发布 valid=false / "
                "INSUFFICIENT_MARKERS（细则要求的无效状态），绝不伪造位姿")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(FieldPose, "/localization/field_pose", 10)
        self.sub = self.create_subscription(
            Image, "/soccer/image_raw", self._on_image, 10)
        self.frame_i = 0
        self.last = None            # (x, y, h, yaw, marker_count, rms, valid, status)
        self.get_logger().info("field_pose_node 已启动，订阅 /soccer/image_raw")

    @staticmethod
    def _abs(p):
        q = Path(str(p))
        return q if q.is_absolute() else WEEK3 / q

    def _on_image(self, msg):
        self.frame_i += 1
        if self.frame_i % self.pose_every != 0:
            self._publish(msg.header.stamp, self.last)
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.marker_model is None:
            self.last = (0.0, 0.0, 0.0, 0.0, 0, 0.0, False,
                         "MARKER_MODEL_NOT_TRAINED")
            self._publish(msg.header.stamp, self.last)
            return

        dets = []
        for r in self.marker_model(frame, verbose=False, conf=self.conf)[0]:
            dets.append({"cls": self.marker_model.names[int(r.boxes.cls)],
                         "box": r.boxes.xyxy[0].tolist(),
                         "conf": float(r.boxes.conf)})
        n = len(dets)
        if n < 4:
            self.last = (0.0, 0.0, 0.0, 0.0, n, 0.0, False,
                         "INSUFFICIENT_MARKERS")
            self._publish(msg.header.stamp, self.last)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        prior = {"h": self.h_prior, "h_sigma": 0.10,
                 "pitch_deg": 14.525, "pitch_sigma": 12.0}
        try:
            pose, _ = self.loc.solve_from_detections(gray, dets, prior=prior)
        except Exception as e:                          # noqa: BLE001
            self.get_logger().warn(f"位姿解算异常：{e}")
            pose = None
        if pose is None or pose.C_F[2] <= 0 or pose.rms_px >= self.residual_thresh:
            rms = pose.rms_px if pose is not None else 0.0
            self.last = (0.0, 0.0, 0.0, 0.0, n, rms, False, "AMBIGUOUS")
        else:
            self.last = (float(pose.C_F[0]), float(pose.C_F[1]),
                         float(pose.C_F[2]), float(pose.yaw_deg),
                         n, float(pose.rms_px), True, "OK")
        self._publish(msg.header.stamp, self.last)

    def _publish(self, stamp, s):
        msg = FieldPose()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_frame"
        if s is None:
            s = (0.0, 0.0, 0.0, 0.0, 0, 0.0, False, "NO_IMAGE_YET")
        x, y, h, yaw, n, rms, valid, status = s
        msg.x_camera_map = x
        msg.y_camera_map = y
        msg.yaw_camera_map = yaw
        # 载体参考原点 = 相机光心（独立移动测试架）
        msg.x_carrier_map = x
        msg.y_carrier_map = y
        msg.yaw_carrier_map = yaw
        msg.valid = valid
        msg.marker_count = n
        msg.residual = rms
        msg.status = status
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FieldPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
