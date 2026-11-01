# -*- coding: utf-8 -*-
"""任务 3：ROS2 发布目标位置并完成订阅验证（30 分）

运行环境（重要）：
  发布端 / 订阅端必须在本机 WSL2 + ROS2 Humble 里运行（rclpy 只存在于 ROS2 环境）：
      source /opt/ros/humble/setup.bash
      source ~/ros2_ws/install/setup.bash
      python3 src/task3.py --role pub     # 终端 A：发布端
      python3 src/task3.py --role sub     # 终端 B：订阅端

  Windows 的普通 Python（例如 cv_practice 环境）里没有 rclpy：脚本会给出提示并退出，
  不会抛出一长串 traceback。只想先验证检测逻辑与消息内容时，可在 Windows 上用：
      python src/task3.py --dry-run
  它不连接 ROS，只把每条“将要发布”的消息按字段打印出来。

数据来源：逐张处理 data/images/ 下的测试图片，按固定频率循环发布；每条消息的
detected/u/v/area 都来自真实 HSV 检测（复用 src/task1.py 的 detect），不是写死的坐标；
未检测到目标时按约定发布 detected=false、u=v=area=0。
路径按脚本自身位置解析，不依赖当前工作目录（任意目录均可运行）。

说明：src/vision_pubsub/ 是基于本文件的 ROS2 包版本（ros2 run 入口，截图使用的就是它），
两者共用同一套阈值与检测逻辑，输出数值一致。
"""
import argparse
import os
import sys

from task1 import CONFIG, IMG_DIR, detect, load_image, rel_path

DEFAULT_TOPIC = "/target_position"

# rclpy / 自定义消息只在 ROS2 环境里可用；Windows 普通 Python 下没有它们。
try:
    import rclpy
    from rclpy.node import Node
    from target_interfaces.msg import TargetPosition
    ROS_OK = True
    ROS_ERROR = None
except ImportError as exc:  # noqa: F841  (在 Windows 上属于预期情况)
    rclpy = None
    Node = object
    TargetPosition = None
    ROS_OK = False
    ROS_ERROR = exc


def detect_image(image_dir, name):
    """对一张图跑任务 1 的检测，返回 (detected, u, v, area)。"""
    cfg = CONFIG.get(name, CONFIG["obj1.jpg"])
    img = load_image(os.path.join(image_dir, name))
    if img is None:
        return False, 0.0, 0.0, 0.0
    detected, u, v, area, _mask, _cands = detect(img, cfg["ranges"], cfg["min_area"])
    return detected, u, v, area


def list_images(image_dir):
    """列出目录中的图片；目录不存在或没有图片时给出明确错误。"""
    if not os.path.isdir(image_dir):
        raise SystemExit(f"[错误] 找不到图片目录 {rel_path(image_dir)}（可用 --image-dir 指定）")
    images = [f for f in sorted(os.listdir(image_dir))
              if f.lower().endswith((".jpg", ".jpeg", ".png"))]
    if not images:
        raise SystemExit(f"[错误] {rel_path(image_dir)} 里没有图片")
    return images


if ROS_OK:
    class DetectorPublisher(Node):
        """发布端：逐张处理测试图片，把检测结果发到话题上。"""

        def __init__(self, image_dir, topic, rate):
            super().__init__("detector_node")
            self.pub = self.create_publisher(TargetPosition, topic, 10)
            self.image_dir = image_dir
            self.images = list_images(image_dir)
            self.idx = 0
            self.count = 0
            self.get_logger().info(
                f"共 {len(self.images)} 张图，{rate:g}Hz 循环发布到 {topic}")
            self.timer = self.create_timer(rate, self.tick)

        def tick(self):
            name = self.images[self.idx % len(self.images)]
            detected, u, v, area = detect_image(self.image_dir, name)
            msg = TargetPosition()
            msg.detected = detected
            msg.u = u
            msg.v = v
            msg.area = area
            self.pub.publish(msg)
            self.count += 1
            self.get_logger().info(
                f"[{self.count}] {name}: detected={msg.detected} "
                f"u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f}")
            self.idx += 1


    class TargetSubscriber(Node):
        """订阅端：订阅话题并打印收到的目标位置。"""

        def __init__(self, topic):
            super().__init__("subscriber_node")
            self.count = 0
            self.create_subscription(TargetPosition, topic, self.callback, 10)
            self.get_logger().info(f"已订阅 {topic}")

        def callback(self, msg):
            self.count += 1
            self.get_logger().info(
                f"[{self.count}] 收到: detected={msg.detected} "
                f"u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.1f}")


def dry_run(image_dir, topic, rate):
    """不连接 ROS，只按消息字段打印将要发布的内容（用于 Windows 上验证检测部分）。"""
    images = list_images(image_dir)
    print("=== 任务3 dry-run：只验证检测与消息内容（不连接 ROS2） ===")
    print(f"话题: {topic}    频率: {rate:g}Hz    图片目录: {rel_path(image_dir)}\n")
    for name in images:
        detected, u, v, area = detect_image(image_dir, name)
        print(f"[dry-run] {name} -> TargetPosition(detected={detected}, "
              f"u={u:.1f}, v={v:.1f}, area={area:.0f})")
    print(f"\n共 {len(images)} 条。真实发布/订阅请在 WSL + ROS2 里运行："
          f"python3 src/task3.py --role pub / --role sub")


def main():
    parser = argparse.ArgumentParser(description="任务3：ROS2 目标位置发布/订阅")
    parser.add_argument("--role", choices=("pub", "sub"), default="pub",
                        help="pub=发布端，sub=订阅端")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="话题名")
    parser.add_argument("--image-dir", default=IMG_DIR, help="发布端图片目录")
    parser.add_argument("--rate", type=float, default=1.0, help="发布频率（Hz）")
    parser.add_argument("--dry-run", action="store_true",
                        help="不连接 ROS，只打印将要发布的消息（Windows 上验证检测用）")
    args = parser.parse_args()

    if args.dry_run:
        dry_run(args.image_dir, args.topic, args.rate)
        return

    if not ROS_OK:
        print("[提示] 当前 Python 环境没有 ROS2（rclpy），task3.py 的发布/订阅只能在 "
              "WSL2 + Ubuntu 22.04 + ROS2 Humble 里运行：")
        print("   source /opt/ros/humble/setup.bash")
        print("   source ~/ros2_ws/install/setup.bash")
        print("   python3 src/task3.py --role pub   # 终端 A")
        print("   python3 src/task3.py --role sub   # 终端 B")
        print(f"   （当前环境报错：{ROS_ERROR}）")
        print("   只想先验证检测逻辑：python src/task3.py --dry-run")
        sys.exit(1)

    if args.role == "pub":
        list_images(args.image_dir)  # 提前给出明确错误
    rclpy.init()
    if args.role == "pub":
        node = DetectorPublisher(args.image_dir, args.topic, args.rate)
    else:
        node = TargetSubscriber(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
