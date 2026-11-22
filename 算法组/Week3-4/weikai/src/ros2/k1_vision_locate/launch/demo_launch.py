"""demo_launch.py —— 一键启动 k1_vision_node（足球检测 + 场地自定位）

节点必须用视觉 venv 的解释器运行（rclpy 来自 source 过的 ROS 环境，
ultralytics/torch 来自 venv，同为 Python 3.10）。本 launch 已自动处理。

用法（WSL）：
  source /opt/ros/humble/setup.bash
  source ~/vision_ws/ros2_ws/install/setup.bash
  # 实时相机（先在 Windows 侧启动 win_cam_bridge）：
  ros2 launch k1_vision_locate demo_launch.py
  # 视频文件自测：
  ros2 launch k1_vision_locate demo_launch.py source:=/mnt/d/captures/M10.mp4

话题：
  /soccer/ball              cquick_interfaces/msg/BallDetection
  /localization/field_pose  cquick_interfaces/msg/FieldPose
"""
import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

VENV_PY_DEFAULT = ("/home/weikai/vision_ws/k1_soccer_vision/.venv/"
                   "bin/python")


def _make_process(context, *args, **kwargs):
    pkg_prefix = get_package_prefix("k1_vision_locate")
    script = os.path.join(pkg_prefix, "lib", "k1_vision_locate", "vision_node")
    params = os.path.join(get_package_share_directory("k1_vision_locate"),
                          "config", "params.yaml")
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    cmd = [
        cfg("venv_python"), script,
        "--ros-args",
        # 先读参数文件，再用 -p 覆盖 launch 参数（rcl 按出现顺序，后者生效）
        "--params-file", params,
        "-p", "source:=" + cfg("source"),
        "-p", "ball_weights:=" + cfg("ball_weights"),
        "-p", "marker_weights:=" + cfg("marker_weights"),
    ]
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="net://127.0.0.1:8090"),
        DeclareLaunchArgument(
            "ball_weights",
            default_value="/home/weikai/vision_ws/k1_soccer_vision/runs/"
                          "ball_yolov8n_aug2/weights/best.pt"),
        DeclareLaunchArgument(
            "marker_weights",
            default_value="/mnt/d/captures/landmarks/weights/"
                          "best_facing_v3.pt"),
        DeclareLaunchArgument("venv_python", default_value=VENV_PY_DEFAULT),
        OpaqueFunction(function=_make_process),
    ])
