"""启动定位节点。

用法：
    ros2 launch k1_vision_ros2 vision.launch.py
    ros2 launch k1_vision_ros2 vision.launch.py camera_url:=http://localhost:8000/stream

本包只封装任务 2（场地自定位）。任务 1 的足球检测仍由
src/detect/detect_live.py 的实时分屏程序提供，未做 ROS2 封装。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    url = LaunchConfiguration("camera_url")

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_url",
            default_value="http://localhost:8000/stream",
            description="MJPEG 推流地址（Windows 侧 win_camera_server.py 提供）",
        ),
        Node(
            package="k1_vision_ros2",
            executable="pose_node",
            name="k1_vision_pose",
            output="screen",
            parameters=[{"camera_url": url}],
        ),
    ])
