# ROS2 节点：足球检测，发布 /ball/bbox 与 /ball/status
import os
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from ultralytics import YOLO

PKG_DIR = os.path.dirname(os.path.abspath(__file__))


def _find_root():
    roots = [PKG_DIR, os.getcwd()]
    for r in list(roots):
        cur = r
        for _ in range(8):
            roots.append(cur)
            nxt = os.path.dirname(cur)
            if nxt == cur:
                break
            cur = nxt
    for r in roots:
        if os.path.isfile(os.path.join(r, "config", "markers_c0.yaml")):
            return r
    return os.getcwd()


PROJ_ROOT = _find_root()
DEFAULT_MODEL = os.path.join(PROJ_ROOT, "runs", "detect", "train4", "weights", "best.pt")


class BallNode(Node):

    def __init__(self):
        super().__init__("k1_vision_ball")
        self.declare_parameter("camera_url", "http://localhost:8000/stream")
        self.declare_parameter("model", DEFAULT_MODEL)
        self.declare_parameter("conf", 0.4)
        url = self.get_parameter("camera_url").value
        path = self.get_parameter("model").value or DEFAULT_MODEL
        self.conf = float(self.get_parameter("conf").value)
        self.model = YOLO(path)
        self.cap = cv2.VideoCapture(url)
        self.pub = self.create_publisher(Float64MultiArray, "/ball/bbox", 10)
        self.pub_st = self.create_publisher(String, "/ball/status", 10)
        self.timer = self.create_timer(0.02, self.tick)
        self.n = 0
        self.t0 = time.time()
        self.get_logger().info(f"BallNode 启动，模型 {path}，图像源 {url}")

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        if frame.shape[1] != 1280:
            frame = cv2.resize(frame, (1280, 720))

        res = self.model(frame, conf=self.conf, verbose=False)[0]
        boxes = []
        for b in res.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            boxes.append([x1, y1, x2, y2, float(b.conf[0]),
                          (x1 + x2) / 2, (y1 + y2) / 2])
        boxes.sort(key=lambda r: -r[4])

        msg = Float64MultiArray()
        msg.data = [float(v) for v in boxes[0]] if boxes else []
        self.pub.publish(msg)

        self.n += 1
        if self.n % 30 == 0:
            s = String()
            s.data = f"count={len(boxes)} fps={30 / max(time.time() - self.t0, 1e-6):.1f}"
            self.pub_st.publish(s)
            self.get_logger().info(s.data)
            self.t0 = time.time()

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = BallNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
