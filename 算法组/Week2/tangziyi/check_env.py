#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""环境自检：Python / OpenCV / ROS2 / 自定义消息是否就绪。

运行：  python3 check_env.py
"""

import os
import shutil
import sys


def show(key, value):
    print("  %-16s %s" % (key, value))


def main():
    print("=" * 66)
    print("CQuick 视觉组 第 2 周 · 环境自检")
    print("=" * 66)

    show("Python", "%s  (%s)" % (sys.version.split()[0], sys.executable))

    try:
        import cv2
        import numpy
        show("OpenCV", cv2.__version__)
        show("NumPy", numpy.__version__)
    except ImportError:
        show("OpenCV", "未安装  ->  sudo apt install python3-opencv")

    show("ROS_DISTRO", os.environ.get("ROS_DISTRO") or "（空，没有 source）")
    show("ros2", shutil.which("ros2") or "未找到（命令不在 PATH 里）")

    try:
        import rclpy  # noqa: F401
        show("rclpy", "可导入")
    except ImportError:
        show("rclpy", "不可导入（ROS2 的 Python 环境没准备好）")

    try:
        from cquick_interfaces.msg import TargetPosition  # noqa: F401
        show("TargetPosition", "可导入")
    except ImportError:
        show("TargetPosition", "不可导入（接口包没编译或没 source）")

    print("-" * 66)
    ok = (os.environ.get("ROS_DISTRO") == "humble"
          and shutil.which("ros2") is not None
          and "rclpy" in sys.modules)
    if ok:
        print("结论：ROS2 Humble 已安装并已 source，可以运行 task3.py。")
    else:
        print("结论：ROS2 Humble 还没装好（或当前终端没 source），")
        print("      task1.py / task2.py 不受影响，task3.py 需要按 README 第 1 节先装 ROS2。")
    print("=" * 66)


if __name__ == "__main__":
    main()
