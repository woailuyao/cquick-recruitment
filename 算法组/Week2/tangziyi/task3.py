#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 3：把任务 1 的检测结果通过 ROS2 话题发出去，再由订阅节点打印。

一个文件里写了两个节点，用命令行参数选择跑哪个：
    终端 A:  python3 task3.py pub        # 发布节点，发到 /target_position
    终端 B:  python3 task3.py sub        # 订阅节点，打印 detected / u / v / area

前提：
    1) 已安装 ROS2 Humble 并 source /opt/ros/humble/setup.bash
    2) 已编译 cquick_interfaces 接口包并 source 该工作空间的 install/setup.bash
       （编译方法见 README 第 4 节）

检测部分直接复用 task1.py 的 detect()，保证发出去的是真实检测结果，不是写死的坐标。
"""

import glob
import os
import sys

try:
    import rclpy
    from rclpy.node import Node
    from cquick_interfaces.msg import TargetPosition
except ImportError as exc:
    print("[错误] %s" % exc)
    print("没找到 rclpy 或 cquick_interfaces，说明 ROS2 环境还没准备好：")
    print("  1) source /opt/ros/humble/setup.bash")
    print("  2) 编译接口包后 source ~/ros2_ws/install/setup.bash")
    print("  3) 再运行 python3 task3.py pub / sub")
    sys.exit(1)

from task1 import IMAGE_DIR, detect, imread_cn, list_images

TOPIC = "/target_position"


def load_frames(image_dir):
    """读入图片 → 逐张跑任务1的检测 → 缓存成待发布的帧。"""
    frames = []
    images = []
    for path in list_images(image_dir):
        img = imread_cn(path)
        if img is None:
            print("  读取失败，跳过：%s" % path)
            continue
        images.append((os.path.basename(path), img))

    for name, img in images:
        res = detect(img)
        frames.append((name, res))
        print("  %-8s detected=%-5s u=%7.1f v=%7.1f area=%8.0f"
              % (name, res["detected"], res["u"], res["v"], res["area"]))

    # 再补一帧“没有球”的负样本：裁第一张图的左下角草地，
    # 用来验证“没检测到目标”时发的是 detected=false，而不是把 (0,0) 当目标。
    if images:
        name, img = images[0]
        h, w = img.shape[:2]
        crop = img[int(h * 0.75):, :int(w * 0.5)]
        res = detect(crop)
        frames.append((name + "[no-target]", res))
        print("  %-8s detected=%-5s  （负样本，用于验证未识别到目标的情况）"
              % (name + "[no-target]", res["detected"]))
    return frames


class TargetPublisher(Node):
    """发布节点：按固定频率把检测结果发到 /target_position。"""

    def __init__(self):
        super().__init__("target_publisher")
        self.declare_parameter("image_dir", IMAGE_DIR)
        self.declare_parameter("publish_rate", 2.0)

        image_dir = self.get_parameter("image_dir").value
        rate = float(self.get_parameter("publish_rate").value)

        print("图片目录：%s" % image_dir)
        self.frames = load_frames(image_dir)
        if not self.frames:
            self.get_logger().error("没有可发布的帧，检查 image 目录")
            raise SystemExit(1)

        self.publisher = self.create_publisher(TargetPosition, TOPIC, 10)
        self.index = 0
        self.count = 0
        self.timer = self.create_timer(1.0 / max(0.2, rate), self.tick)

        self.get_logger().info("发布话题 %s，类型 cquick_interfaces/msg/TargetPosition，"
                               "频率 %.1f Hz，共 %d 帧轮流发布"
                               % (TOPIC, rate, len(self.frames)))

    def tick(self):
        name, res = self.frames[self.index]
        msg = TargetPosition()
        msg.detected = bool(res["detected"])
        msg.u = float(res["u"])
        msg.v = float(res["v"])
        msg.area = float(res["area"])
        self.publisher.publish(msg)

        self.count += 1
        self.get_logger().info("#%03d 来源=%-18s detected=%-5s u=%7.1f v=%7.1f area=%8.0f"
                               % (self.count, name, msg.detected, msg.u, msg.v, msg.area))
        self.index = (self.index + 1) % len(self.frames)


class TargetSubscriber(Node):
    """订阅节点：收到什么就打印什么。"""

    def __init__(self):
        super().__init__("target_subscriber")
        self.count = 0
        self.valid = 0
        self.subscription = self.create_subscription(
            TargetPosition, TOPIC, self.callback, 10)
        self.get_logger().info("订阅 %s，等待消息……" % TOPIC)

    def callback(self, msg):
        self.count += 1
        if msg.detected:
            self.valid += 1
        self.get_logger().info("#%03d detected=%-5s u=%7.1f v=%7.1f area=%8.0f  %s"
                               % (self.count, msg.detected, msg.u, msg.v, msg.area,
                                  "TARGET" if msg.detected else "NO-TARGET"))

    def summary(self):
        return ("共接收 %d 条，detected=True %d 条，detected=False %d 条"
                % (self.count, self.valid, self.count - self.valid))


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("pub", "sub"):
        print("用法：python3 task3.py pub   # 发布节点")
        print("      python3 task3.py sub   # 订阅节点")
        return

    rclpy.init()
    node = TargetPublisher() if sys.argv[1] == "pub" else TargetSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if isinstance(node, TargetSubscriber):
            print("\n[统计] %s" % node.summary())
        elif isinstance(node, TargetPublisher):
            print("\n[统计] 共发布 %d 条消息" % node.count)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
