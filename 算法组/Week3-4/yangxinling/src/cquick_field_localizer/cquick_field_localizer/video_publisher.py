#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video_publisher.py —— 把视频文件 / USB 相机当成"标准相机"发到 /image_raw

为什么需要它：
    1. 现场没有相机、或相机被占用时，仍能用录好的视频完整演示整条链路；
    2. 重复性测试：同一个视频每次跑出来的结果必须一样，方便写测试报告；
    3. 录 B 站演示视频时，画面来源可控，不用手抖；
    4. 这台 WSL 机器上没有装 usb_cam，用本节点直接开 /dev/videoN 就够，
       少一个 apt 依赖，launch 也就不会因为"找不到 usb_cam"起动失败。

用法：
    # 播视频
    ros2 run cquick_field_localizer video_publisher --ros-args \
        -p video:="video/场地/B点.avi" -p fps:=30.0 -p loop:=true
    # 开相机（device>=0 时忽略 video）
    ros2 run cquick_field_localizer video_publisher --ros-args \
        -p device:=0 -p width:=1280 -p height:=720

参数：
    video    视频文件路径（相对路径先按当前目录、再按 $CQUICK_REPO_ROOT 解析）
    device   USB 相机编号（-1 = 用 video）
    width    相机分辨率（仅 device>=0 生效）
    height   相机分辨率（仅 device>=0 生效）
    fps      发布帧率（<=0 表示按视频原帧率 / 相机 30）
    loop     播完是否循环（仅视频模式）
    topic    话题名（默认 /image_raw）
    start    从第几帧开始（仅视频模式）
"""

import os
import sys

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from .rosimg import bgr_to_imgmsg


def _resolve_video(video):
    """相对路径：先当前目录，再 $CQUICK_REPO_ROOT，最后仓库根（本文件的 ../../../..）。"""
    if os.path.isabs(video) and os.path.isfile(video):
        return video
    if os.path.isfile(video):
        return os.path.abspath(video)
    cands = []
    root = os.environ.get("CQUICK_REPO_ROOT", "")
    if root:
        cands.append(os.path.join(root, video))
    here = os.path.dirname(os.path.abspath(__file__))
    cands.append(os.path.join(here, "..", "..", "..", video))
    for c in cands:
        c = os.path.abspath(c)
        if os.path.isfile(c):
            return c
    return None


class VideoPublisher(Node):
    def __init__(self):
        super().__init__("video_publisher")
        self.declare_parameter("video", "")
        self.declare_parameter("device", -1)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 0.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("topic", "/image_raw")
        self.declare_parameter("start", 0)

        video = self.get_parameter("video").value
        device = int(self.get_parameter("device").value)
        fps_param = float(self.get_parameter("fps").value)
        self.loop = bool(self.get_parameter("loop").value)

        if device >= 0:
            # ---------------- 相机模式 ----------------
            self.mode = "camera"
            self.cap = cv2.VideoCapture(device)
            if not self.cap.isOpened():
                raise RuntimeError("打不开相机 /dev/video%d（检查 USB 连接与权限）" % device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("width").value))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("height").value))
            if fps_param > 0:
                self.cap.set(cv2.CAP_PROP_FPS, fps_param)
            self.fps = fps_param if fps_param > 0 else 30.0
            self.n_frames = -1
            self.label = "/dev/video%d" % device
        else:
            # ---------------- 视频模式 ----------------
            self.mode = "video"
            if not video:
                raise RuntimeError("必须给 -p video:=<路径> 或 -p device:=<编号>")
            if video.startswith(("http://", "https://")):
                # 网络视频流（Windows 侧 tools/cam_server.py 推的 MJPEG），绕开 usbip 直通
                from .netcam import NetCamera
                self.cap = NetCamera(video)
                if not self.cap.isOpened():
                    raise RuntimeError("打不开网络视频流：%s" % video)
                self.fps = fps_param if fps_param > 0 else 30.0
                self.n_frames = 0
                self.frame_id = 0
                self.label = video
                self._is_net = True
            else:
                path = _resolve_video(video)
                if path is None:
                    raise RuntimeError("视频不存在：%s（可设 CQUICK_REPO_ROOT 指向仓库根）" % video)
                self.cap = cv2.VideoCapture(path)
                if not self.cap.isOpened():
                    raise RuntimeError("打不开视频：%s" % path)
                src_fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
                self.fps = fps_param if fps_param > 0 else src_fps
                self.n_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
                start = int(self.get_parameter("start").value)
                if start > 0:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, start)
                self.frame_id = start
                self.label = os.path.basename(path)

        if not hasattr(self, "frame_id"):
            self.frame_id = 0
        topic = self.get_parameter("topic").value
        self.pub = self.create_publisher(Image, topic, 2)
        self.timer = self.create_timer(1.0 / max(self.fps, 1e-3), self.tick)
        self.get_logger().info("发布源 %s（%.1f fps）→ %s" % (self.label, self.fps, topic))

    def tick(self):
        # Ctrl-C 后 rclpy 先 shutdown，定时器可能还会再触发一次；此时 publish 会抛
        # RCLError("publisher's context is invalid")，先判断再发。
        if not rclpy.ok():
            self.timer.cancel()
            return
        ok, frame = self.cap.read()
        if not ok:
            if self.mode == "video" and self.loop and not getattr(self, "_is_net", False):
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.frame_id = 0
                ok, frame = self.cap.read()
            if not ok:
                self.get_logger().info("数据源结束，停止发布")
                self.timer.cancel()
                return
        self.frame_id += 1
        msg = bgr_to_imgmsg(frame, stamp=self.get_clock().now().to_msg(), frame_id="camera")
        try:
            self.pub.publish(msg)
        except Exception:
            self.timer.cancel()

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
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
