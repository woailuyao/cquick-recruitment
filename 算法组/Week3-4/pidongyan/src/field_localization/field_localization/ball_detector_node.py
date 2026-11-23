"""ROS2 节点 ball_detector_node：真实相机 → YOLOv8n → /soccer/ball。

同时把原始帧发布到 /soccer/image_raw（sensor_msgs/Image），
供 field_pose_node 复用 —— 单 USB 相机只能被一个进程打开，
所以**只有本节点开相机**，定位节点订阅图像话题而不是自己再开一次。

⚠️ 开发机是 Windows 11（无 ROS2 Humble），本文件未在实机跑通，
   20 号下午到实验室 Ubuntu 机按 src/README.md 验证。

查话题：ros2 topic echo /soccer/ball
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_interfaces.msg import BallDetection


class BallDetectorNode(Node):
    def __init__(self):
        super().__init__("ball_detector_node")
        self.declare_parameter("model_path", "best_train_v4.pt")
        self.declare_parameter("source", "0")           # 0=真实相机索引
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)

        model_path = self.get_parameter("model_path").value
        src = str(self.get_parameter("source").value)
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        fps = float(self.get_parameter("fps").value)

        from ultralytics import YOLO
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.pub_ball = self.create_publisher(BallDetection, "/soccer/ball", 10)
        self.pub_img = self.create_publisher(Image, "/soccer/image_raw", 10)

        if src.isdigit():
            self.cap = cv2.VideoCapture(int(src), cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("width").value))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("height").value))
            self.cap.set(cv2.CAP_PROP_FPS, fps)
        else:
            # 非数字 source 仅供调试，验收必须用真实相机
            self.get_logger().warn("source 是视频文件，属调试模式，不得作为实时验收证据")
            self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开相机/视频源：{src}")

        # 预热，避免首帧 CUDA kernel 加载拖慢统计
        self.model.predict(np.zeros((720, 1280, 3), np.uint8),
                           imgsz=self.imgsz, verbose=False)
        self.timer = self.create_timer(1.0 / fps, self._tick)
        self.get_logger().info("ball_detector_node 已启动，发布 /soccer/ball 与 /soccer/image_raw")

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("取帧失败", throttle_duration_sec=2.0)
            return
        stamp = self.get_clock().now().to_msg()

        msg = BallDetection()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_frame"
        msg.detected = False
        r = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf,
                                verbose=False)[0]
        if r.boxes is not None and len(r.boxes):
            xyxy = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            k = int(np.argmax(confs))
            x1, y1, x2, y2 = [float(v) for v in xyxy[k]]
            msg.detected = True
            msg.conf = float(confs[k])
            msg.x1, msg.y1, msg.x2, msg.y2 = int(x1), int(y1), int(x2), int(y2)
            msg.center_u, msg.center_v = (x1 + x2) / 2, (y1 + y2) / 2
        self.pub_ball.publish(msg)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = "camera_frame"
        self.pub_img.publish(img_msg)

    def destroy_node(self):
        if hasattr(self, "cap"):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
