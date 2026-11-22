# -*- coding: utf-8 -*-
"""ROS2 入口：复用 split_view 的完整流程（相机 / 足球 / 标志 / 地图 / 分屏），
并把结果发布到 /soccer/ball 与 /localization/field_pose。

要处理两类 launch 噪声：
1) `--ros-args -r __node:=...`        -> 用 rclpy.utilities.remove_ros_args 剥掉
2) 空值的 launch 参数（如 clicks:=）   -> 会变成 "--clicks --loop"，
   argparse 认为 --clicks 缺参数而报错 -> 这里把"没有值的选项"直接丢弃
"""
import os
import sys

VALUE_OPTS = {"--source", "--clicks", "--save-clicks", "--camera", "--field",
              "--record", "--conf", "--max-frames", "--start-frame"}


def _strip_ros_args(argv):
    try:
        from rclpy.utilities import remove_ros_args
        return remove_ros_args(args=["x"] + list(argv))[1:]
    except Exception:
        if "--ros-args" in argv:
            argv = argv[: argv.index("--ros-args")]
        return [a for a in argv if not a.startswith("__")]


def _clean(argv):
    """去掉 ROS 参数，并丢弃没有取值的选项（launch 传空值时会出现）。"""
    argv = _strip_ros_args(list(argv))
    out, i = [], 0
    while i < len(argv):
        a = argv[i]
        if a in VALUE_OPTS:
            nxt = argv[i + 1] if i + 1 < len(argv) else None
            if nxt is not None and not nxt.startswith("-"):
                out.extend([a, nxt])
                i += 2
            else:
                i += 1          # 空值 -> 丢弃该选项
            continue
        out.append(a)
        i += 1
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)

    cam_yaml = os.path.join(here, "camera.yaml")
    field_yaml = os.path.join(here, "field.yaml")
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("week34_vision")
        c = os.path.join(share, "config", "camera.yaml")
        f = os.path.join(share, "config", "field.yaml")
        if os.path.exists(c):
            cam_yaml = c
        if os.path.exists(f):
            field_yaml = f
    except Exception:
        pass

    argv = _clean(sys.argv[1:])
    if "--ros2" not in argv:
        argv.append("--ros2")

    sys.argv = ["split_view", "--camera", cam_yaml, "--field", field_yaml] + argv

    import split_view
    split_view.main()


if __name__ == "__main__":
    main()
