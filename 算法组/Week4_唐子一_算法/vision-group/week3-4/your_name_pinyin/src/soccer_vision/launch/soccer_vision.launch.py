"""launch 文件：同时启动足球检测 + 场地定位节点。

兼容 ROS2 Humble 与 Jazzy（Python launch 格式通用）。
用法：
    ros2 launch soccer_vision soccer_vision.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ball_node = Node(
        package="soccer_vision",
        executable="ros2_ball_node.py",
        name="ball_detection_node",
        output="screen",
        parameters=[{"weights": "best.pt", "device": "/dev/video0", "conf": 0.5}],
    )

    localization_node = Node(
        package="soccer_vision",
        executable="ros2_localization_node.py",
        name="field_localization_node",
        output="screen",
        parameters=[
            {
                "field": "config/field.yaml",
                "geometry": "config/field_geometry.yaml",
                "intrinsics": "calibration/camera_intrinsics.yaml",
                "extrinsics": "calibration/camera_extrinsics.yaml",
                "marker_model": "best_marker.pt",
                "device": "/dev/video0",
                "conf": 0.5,
            }
        ],
    )

    return LaunchDescription([ball_node, localization_node])
