#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CQuick 任务二 · 单目场地定位 launch 文件

用法：
    ros2 launch task2_localization task2.launch.py                     # 仿真模式
    ros2 launch task2_localization task2.launch.py use_camera:=true    # 实时相机
    ros2 launch task2_localization task2.launch.py field_yaml:=/abs/path/field.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    try:
        default_yaml = os.path.join(
            get_package_share_directory("task2_localization"), "config", "field.yaml")
    except Exception:
        default_yaml = "config/field.yaml"

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="false",
                              description="true=实时相机，false=离线仿真"),
        DeclareLaunchArgument("field_yaml", default_value=default_yaml,
                              description="field.yaml 路径"),
        DeclareLaunchArgument("pitch_deg", default_value="19.0",
                              description="相机俯仰角（向下为正）"),
        DeclareLaunchArgument("height_m", default_value="0.73",
                              description="相机光心离地高度"),
        DeclareLaunchArgument("residual_max", default_value="0.20",
                              description="残差阈值（米），超过判无效"),
        DeclareLaunchArgument("model_path", default_value="weights/best.pt"),
        DeclareLaunchArgument("source", default_value="0", description="0=相机，或视频路径"),
        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("rate_hz", default_value="10.0",
                              description="发布频率"),

        # 任务一：足球检测（需要权重与视频源，无源时不会报错退出）
        Node(
            package="task2_localization",
            executable="task1_node",
            name="task1_ball_detect",
            output="screen",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "source": LaunchConfiguration("source"),
                "conf": LaunchConfiguration("conf"),
            }],
        ),

        Node(
            package="task2_localization",
            executable="task2_node",
            name="task2_localization",
            output="screen",
            parameters=[{
                "use_camera": LaunchConfiguration("use_camera"),
                "field_yaml": LaunchConfiguration("field_yaml"),
                "pitch_deg": LaunchConfiguration("pitch_deg"),
                "height_m": LaunchConfiguration("height_m"),
                "residual_max": LaunchConfiguration("residual_max"),
                "rate_hz": LaunchConfiguration("rate_hz"),
            }],
        ),
    ])

