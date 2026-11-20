#!/usr/bin/env python3
"""ROS2 节点：订阅相机图像 → 单目场地定位 → 发布 FieldPose。

依赖仓库里的 vision2 包，所以启动前需要把仓库根目录告诉节点：
    export FIELD_ROOT=$HOME/cquick-recruitment/算法组/Week3-4/xiaoyutong
    ros2 launch field_localization field_localization.launch.py

订阅：
    /image_raw            sensor_msgs/Image         （相机原始图像，1280x720）
发布：
    /localization/field_pose   field_localization/FieldPose
    /localization/debug_image  sensor_msgs/CompressedImage （左右分屏，可选）
"""

import math
import os
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from field_localization.msg import FieldPose


def _find_root():
    root = os.environ.get("FIELD_ROOT", "")
    if root and os.path.exists(os.path.join(root, "config", "field.yaml")):
        return root
    d = os.getcwd()
    while True:
        if os.path.exists(os.path.join(d, "config", "field.yaml")):
            return d
        nd = os.path.dirname(d)
        if nd == d:
            break
        d = nd
    return root


def main(args=None):
    root = _find_root()
    if not root or not os.path.exists(os.path.join(root, "config", "field.yaml")):
        print("[错误] 找不到仓库根目录。请先 export FIELD_ROOT=<仓库根目录>")
        sys.exit(1)
    sys.path.insert(0, os.path.join(root, "src", "field_localization"))
    sys.path.insert(0, root)

    from localize_app import Args, Localizer, make_detector

    rclpy.init(args=args)
    node = Node("field_localization")

    weights = node.declare_parameter("weights", "").value
    detector_name = node.declare_parameter("detector", "yolo").value
    publish_debug = node.declare_parameter("publish_debug_image", True).value

    args_ns = Args()
    args_ns.detector = detector_name
    args_ns.weights = weights
    args_ns.conf = 0.45

    loc = Localizer(args_ns)
    det = make_detector(args_ns)
    bridge = CvBridge()

    pub = node.create_publisher(FieldPose, "/localization/field_pose", 10)
    pub_img = node.create_publisher(CompressedImage, "/localization/debug_image", 2) if publish_debug else None

    def on_image(msg: Image):
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        dets = det.detect(frame)
        res, obs, used = loc.process(frame, dets)

        out = FieldPose()
        out.header = msg.header
        if res.get("x") is not None:
            out.x_camera_map = float(res.get("x_f", res["x"]))
            out.y_camera_map = float(res.get("y_f", res["y"]))
            out.yaw_camera_map = float(res.get("yaw_f", res["yaw_deg"]))
            out.x_carrier_map = out.x_camera_map
            out.y_carrier_map = out.y_camera_map
            out.yaw_carrier_map = out.yaw_camera_map
        out.valid = bool(res.get("valid"))
        out.marker_count = int(res.get("marker_count", 0))
        out.residual = float(res.get("residual") or 0.0)
        out.status = str(res.get("status", "lost"))
        pub.publish(out)

        if pub_img is not None:
            canvas = loc.render(frame, dets, res, obs, used)
            ok, buf = __import__("cv2").imencode(".jpg", canvas)
            if ok:
                im = CompressedImage()
                im.header = msg.header
                im.format = "jpeg"
                im.data = buf.tobytes()
                pub_img.publish(im)

    node.create_subscription(Image, "/image_raw", on_image, 5)
    node.get_logger().info("field_localization 已启动：/image_raw → /localization/field_pose")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
