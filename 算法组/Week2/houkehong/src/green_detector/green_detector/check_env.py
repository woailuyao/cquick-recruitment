#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""前置环境自检：一次把「第 2 周」容易踩的环境问题全查一遍。

    方法一：source 过 ROS2 之后直接跑
        ros2 run green_detector check_env
    方法二：不装包也能跑
        python3 src/green_detector/green_detector/check_env.py
"""

from __future__ import annotations

import os
import platform
import sys


def line(title, value="", ok=None):
    flag = "" if ok is None else ("[ OK ]" if ok else "[FAIL]")
    print("%-6s %-22s %s" % (flag, title, value))


def main() -> int:
    print("=" * 62)
    print("第 2 周环境自检  时间: %s" % __import__("datetime").datetime.now()
          .strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 62)

    line("系统", "%s %s" % (platform.system(), platform.release()))
    line("Python", sys.version.split()[0], sys.version_info >= (3, 8))
    line("解释器路径", sys.executable)

    try:
        import cv2
        line("OpenCV (cv2)", cv2.__version__, True)
    except Exception as exc:                       # noqa: BLE001
        line("OpenCV (cv2)", "导入失败: %s" % exc, False)

    try:
        import numpy
        line("NumPy", numpy.__version__, True)
    except Exception as exc:                       # noqa: BLE001
        line("NumPy", "导入失败: %s" % exc, False)

    distro = os.environ.get("ROS_DISTRO", "")
    line("ROS_DISTRO", distro or "(空，说明没 source ROS2)", distro == "humble")

    try:
        import rclpy                               # noqa: F401
        line("rclpy", "可导入", True)
    except Exception as exc:                       # noqa: BLE001
        line("rclpy", "导入失败: %s" % exc, False)

    try:
        from target_interfaces.msg import TargetPosition   # noqa: F401
        line("自定义消息", "TargetPosition 可用", True)
    except Exception as exc:                       # noqa: BLE001
        line("自定义消息", "不可用: %s" % exc, False)

    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("green_detector")
        line("green_detector", share, True)
    except Exception as exc:                       # noqa: BLE001
        line("green_detector", "未安装或未 source: %s" % exc, False)

    print("-" * 62)
    print("提示：cv2 能导入但节点里 import 失败，通常是「系统 python3 有 cv2、ROS2 用的解释器没有」，")
    print("      或反之。用上面打印的解释器路径确认是不是同一个 python。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
