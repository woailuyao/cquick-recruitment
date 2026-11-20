#!/usr/bin/env python3
"""发布节点：读测试图片 → HSV 检测 → 发布 TargetPosition 到 /target_position。

用 1 Hz 定时器循环发 3 张图，跑 12 秒就是 12 条消息，满足"至少连续发布 10 条"。
"""
import glob
from pathlib import Path

import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition

from .detect import detect, load

PERIOD = 1.0        # 发布周期（秒）
TOPIC = "/target_position"


def image_dir():
    """优先用安装到 share 的图片；没装就退回源码目录里的 data/。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        d = Path(get_package_share_directory("target_detector")) / "data"
        if d.is_dir() and any(d.glob("*.jpg")):
            return d
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / "data"


class TargetPublisher(Node):
    def __init__(self):
        super().__init__("target_publisher")
        self.pub = self.create_publisher(TargetPosition, TOPIC, 10)

        self.images = sorted(glob.glob(str(image_dir() / "*.jpg")))
        if not self.images:
            self.get_logger().error(f"一张图都没找到，检查 data/ 目录：{image_dir()}")

        self.idx = 0
        self.count = 0
        self.timer = self.create_timer(PERIOD, self.tick)
        self.get_logger().info(
            f"发布节点启动：话题 {TOPIC}，周期 {PERIOD}s，图片 {len(self.images)} 张"
        )

    def tick(self):
        if not self.images:
            return

        path = self.images[self.idx % len(self.images)]   # 循环取图
        self.idx += 1
        res = detect(load(path))

        msg = TargetPosition()
        if res is None:
            # 没检测到就明确标 False，坐标填 0；绝不拿 0 当"有效坐标"用
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
            detail = "未检测到目标"
        else:
            u, v, area = res
            msg.detected = True
            msg.u = float(u)
            msg.v = float(v)
            msg.area = float(area)
            detail = f"u={msg.u:.1f}  v={msg.v:.1f}  area={msg.area:.0f}"

        self.pub.publish(msg)
        self.count += 1
        self.get_logger().info(
            f"[{self.count:02d}] {Path(path).name}  detected={msg.detected}  {detail}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    try:
        rclpy.spin(node)          # 不 spin 定时器不会触发
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
