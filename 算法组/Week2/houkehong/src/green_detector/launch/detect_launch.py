# -*- coding: utf-8 -*-
"""一次拉起「发布节点 + 订阅节点」，方便截分屏图。

    ros2 launch green_detector detect_launch.py
    ros2 launch green_detector detect_launch.py image_dir:=/home/hkh/pics
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("green_detector")
    params_file = os.path.join(share, "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "image_dir", default_value="",
            description="测试图片目录，留空则用包内 share/green_detector/test_images"),
        Node(
            package="green_detector",
            executable="publish_target",
            name="target_publisher",
            output="screen",
            parameters=[params_file, {
                "image_dir": LaunchConfiguration("image_dir"),
            }],
        ),
        Node(
            package="green_detector",
            executable="subscribe_target",
            name="target_subscriber",
            output="screen",
            emulate_tty=True,
        ),
    ])
