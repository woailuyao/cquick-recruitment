#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 2 周任务 3：发布节点 —— 把视觉检测结果发到 /target_position 话题。

对应细则 3.2：
    - 发布节点把检测结果发到 /target_position（消息类型 vision_interfaces/msg/TargetPosition）
    - 数据来源必须说清楚：本节点逐张读取图片目录，先跑任务 1 的检测，再把结果发出去
    - 先用固定坐标验证通信：加 --fixed 参数即可（发布写死的 320,240，
      每 5 条插一条 detected=False 演示"无效帧"的表达方式）
    - 最终验收必须发真实检测结果：默认模式就是真检测（细则 3.2 最后一条）

设计说明（答辩可能会问）：
    1. 检测逻辑和 ROS 逻辑彻底分离：
       DetectionSource 类只管"给定图片 -> 返回 (detected,u,v,area)"，
       TargetPublisher 只管"按频率把结果发出去"。
       好处：可以 --dry-run 在没有 ROS2 的机器（比如 Windows）上单测数据链路。
    2. 图片在启动时一次性检测完并缓存：测试图是静态的，没必要每帧重算，
       发布定时器里只做"取结果 -> 填消息 -> publish"，回调耗时接近 0，频率稳。
    3. 循环发布 len(图片) x repeat 条，默认 repeat=3，5 张图 = 15 条 >= 10 条，
       满足"至少连续发布 10 条消息"的验收要求。

Ubuntu 运行（开两个终端，都要先 source）：
    终端1: source /opt/ros/humble/setup.bash
           source ~/ros2_ws/install/setup.bash
           ros2 run vision_detector target_subscriber
    终端2: ros2 run vision_detector target_publisher --image-dir ~/ros2_ws/photos

Windows 无 ROS 时自测数据链路（不需要 rclpy）：
    python target_publisher.py --dry-run --image-dir D:/CQU/CQuick
"""
from __future__ import annotations

import argparse
from pathlib import Path

# ---------------------------------------------------------------------------
# ROS 依赖允许缺失：没有 ROS2 时仍可用 --dry-run 验证检测 -> 消息的整条数据链路。
# 这是刻意的容错设计，不要去掉 try。
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node
    from vision_interfaces.msg import TargetPosition      # colcon 编译接口包后才有
    ROS_OK = True
except ImportError:                                        # Windows / 未 source 环境
    from dataclasses import dataclass

    @dataclass
    class TargetPosition:                                  # 本地占位类，字段与 .msg 一一对应
        detected: bool = False
        u: float = 0.0
        v: float = 0.0
        area: float = 0.0

    Node = object                                          # 占位，保证类定义不报错
    rclpy = None
    ROS_OK = False

# 导入任务 1 的检测模块：包内运行（ros2 run）和脚本直跑两种方式都要能 import
try:
    from vision_detector.detect_color import detect_target
except ImportError:
    from detect_color import detect_target

IMAGE_SUFFIX = {".jpg", ".jpeg", ".png", ".bmp"}


class DetectionSource:
    """数据源：把"一批图片"变成"一条条检测结果"。

    职责单一：只负责检测，不碰任何 ROS 概念 —— 这样换掉 ROS 部分时它一行不用改。
    """

    def __init__(self, image_dir: Path, color: str = "red", min_area: float = 800.0,
                 do_close: bool = True, ksize: int = 21, center: str = "moments"):
        self.color = color
        self.items: list[tuple[str, bool, float, float, float, str]] = []  # (名字, detected,u,v,area,说明)

        images = sorted(p for p in image_dir.iterdir() if p.suffix.lower() in IMAGE_SUFFIX)
        if not images:
            raise FileNotFoundError(f"{image_dir} 下没有图片")

        # 启动时把所有图片检测一遍并缓存（测试图是静态的，没必要发布时重算）
        for path in images:
            bgr = self._imread(path)
            res, _ = detect_target(bgr, color=color, min_area=min_area,
                                   do_close=do_close, ksize=ksize, center_method=center)
            self.items.append((path.name, *res.as_tuple(), res.message))

    @staticmethod
    def _imread(path: Path):
        import cv2
        # 判空是必须的：cv2.imread 失败返回 None 而不是抛异常
        bgr = cv2.imread(str(path))
        if bgr is None:
            raise IOError(f"图片读取失败：{path}")
        return bgr

    def __len__(self) -> int:
        return len(self.items)

    def get(self, idx: int) -> tuple[str, bool, float, float, float, str]:
        return self.items[idx % len(self.items)]        # 取模 => 自动循环发布


class TargetPublisher(Node):
    """ROS2 发布节点：定时把检测结果发到 /target_position。"""

    def __init__(self, source: DetectionSource, rate_hz: float, repeat: int):
        # 节点名：ros2 node list 里看到的 ID，保持小写下划线风格
        super().__init__("target_publisher")

        # QoS depth=10：发布队列长度。发得比订阅端处理快时最多缓存 10 条，
        # 这是 ROS2 默认值，够用；太小可能丢消息。
        self.pub = self.create_publisher(TargetPosition, "target_position", 10)

        self.source = source
        self.repeat = repeat
        self.msg_seq = 0            # 已发布条数（日志里自证 >= 10 条）
        self.cur = 0                # 当前发到第几张图

        # 定时器：每 1/rate 秒触发一次 tick()，相当于 "while + sleep + publish" 的 ROS 版
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        total = len(source) * repeat
        self.get_logger().info(
            f"开始发布：{len(source)} 张图 x {repeat} 遍 = {total} 条消息，"
            f"频率 {rate_hz} Hz，话题 /target_position")

    def tick(self) -> None:
        """定时器回调：取一条结果 -> 填消息 -> 发布。"""
        if self.cur >= len(self.source) * self.repeat:
            self.get_logger().info(f"发布完成：共 {self.msg_seq} 条。Ctrl+C 退出。")
            self.timer.cancel()                     # 停掉定时器，节点保持存活
            return

        name, detected, u, v, area, note = self.source.get(self.cur)
        self.cur += 1

        # 实例化消息（由 .msg 编译生成的类），逐字段填值
        msg = TargetPosition()
        msg.detected = detected
        msg.u = float(u)
        msg.v = float(v)
        msg.area = float(area)

        self.pub.publish(msg)
        self.msg_seq += 1
        # 日志同时带序号和内容：截图里能同时证明"发了"和"发的什么"
        self.get_logger().info(
            f"[{self.msg_seq:03d}] {name}  detected={detected} u={u:.1f} "
            f"v={v:.1f} area={area:.0f}   ({note})")


class FixedPublisher(Node):
    """固定坐标发布节点（--fixed）：先用写死的数据验证通信链路，与检测解耦。"""

    FIXED = (320.0, 240.0, 1234.0)      # 写死的 (u, v, area)

    def __init__(self, total: int, rate_hz: float):
        super().__init__("target_publisher")
        self.pub = self.create_publisher(TargetPosition, "target_position", 10)
        self.total = total
        self.i = 0
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(f"固定坐标测试模式：将发布 {total} 条，频率 {rate_hz} Hz")

    def tick(self) -> None:
        if self.i >= self.total:
            self.get_logger().info(f"发布完成：共 {self.i} 条。Ctrl+C 退出。")
            self.timer.cancel()
            return
        u, v, a = self.FIXED
        msg = TargetPosition()
        # 每 5 条插一条 detected=False，演示"没检测到"的规范表达：
        # u/v/area 填 0 只表示"无意义"，不能理解成目标在左上角
        msg.detected = (self.i % 5 != 4)
        msg.u = u if msg.detected else 0.0
        msg.v = v if msg.detected else 0.0
        msg.area = a if msg.detected else 0.0
        self.pub.publish(msg)
        self.i += 1
        self.get_logger().info(
            f"[{self.i:03d}] detected={msg.detected} u={msg.u:.1f} "
            f"v={msg.v:.1f} area={msg.area:.0f}" + ("" if msg.detected else "  <- 无效帧"))


def _simulate(source: DetectionSource | None, repeat: int, rate_hz: float, fixed: bool) -> None:
    """--dry-run：没有 ROS2 时模拟发布循环，验证数据链路（Windows 上可用）。"""
    print("[dry-run] 未连接 ROS2，以下是将要发布的消息内容（逻辑与真实节点完全一致）：")
    if fixed:
        total, i = 15, 0
        for i in range(total):
            det = (i % 5 != 4)
            u, v, a = (320.0, 240.0, 1234.0) if det else (0.0, 0.0, 0.0)
            print(f"  [{i + 1:03d}] TargetPosition(detected={det}, u={u}, v={v}, area={a})"
                  + ("" if det else "  <- 无效帧"))
        return
    total = len(source) * repeat
    n = 0
    while n < total:
        name, det, u, v, a, note = source.get(n)
        n += 1
        print(f"  [{n:03d}] TargetPosition(detected={det}, u={u:.1f}, v={v:.1f}, "
              f"area={a:.0f})   <- {name} {note}")


def main() -> int:
    parser = argparse.ArgumentParser(description="目标位置发布节点（第 2 周任务 3）")
    parser.add_argument("--image-dir", type=Path, default=Path.home() / "ros2_ws" / "photos",
                        help="图片目录（数据来源，README 必须说明）")
    parser.add_argument("--rate", type=float, default=2.0, help="发布频率 Hz（默认 2）")
    parser.add_argument("--repeat", type=int, default=3, help="图片列表循环几遍（默认 3）")
    parser.add_argument("--color", default="red")
    parser.add_argument("--min-area", type=float, default=800.0)
    parser.add_argument("--ksize", type=int, default=21)
    parser.add_argument("--no-close", action="store_true", help="关闭闭运算")
    parser.add_argument("--center", default="moments", choices=["moments", "rect"])
    parser.add_argument("--fixed", action="store_true",
                        help="固定坐标测试模式（细则允许：先验证通信再接检测）")
    parser.add_argument("--dry-run", action="store_true",
                        help="不启动 ROS，只打印将发布的消息（Windows 自测用）")
    args = parser.parse_args()

    # 固定坐标模式
    if args.fixed:
        if not ROS_OK:
            _simulate(None, 0, args.rate, fixed=True)
            return 0
        rclpy.init()
        node = FixedPublisher(total=15, rate_hz=args.rate)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
        return 0

    # 真实检测模式：先建数据源（此时还没碰 ROS，检测失败可以干净地报错退出）
    try:
        source = DetectionSource(args.image_dir, color=args.color, min_area=args.min_area,
                                 do_close=not args.no_close, ksize=args.ksize,
                                 center=args.center)
    except Exception as e:
        print(f"数据源初始化失败：{e}")
        return 1

    if args.dry_run or not ROS_OK:
        if not ROS_OK and not args.dry_run:
            print("[提示] 检测到本机没有 ROS2（rclpy 不可用），自动进入 dry-run 模式\n")
        _simulate(source, args.repeat, args.rate, fixed=False)
        return 0

    rclpy.init(args=None)                       # 初始化 ROS2 上下文，节点之前必须调
    node = TargetPublisher(source, args.rate, args.repeat)
    try:
        rclpy.spin(node)                        # 进入事件循环，定时器回调在这里被触发
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()                    # 就算异常退出也要收干净，否则下次 init 报错
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
