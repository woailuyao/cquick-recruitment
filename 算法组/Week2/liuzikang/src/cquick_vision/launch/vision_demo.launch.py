#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一条命令同时启动发布节点和订阅节点。

用法（在项目根目录）：
    ros2 launch cquick_vision vision_demo.launch.py
    ros2 launch cquick_vision vision_demo.launch.py image_dir:=/绝对/路径
    ros2 launch cquick_vision vision_demo.launch.py use_fixed_test_data:=true
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_dir = LaunchConfiguration("image_dir")
    publish_rate = LaunchConfiguration("publish_rate")
    use_fixed = LaunchConfiguration("use_fixed_test_data")
    expected = LaunchConfiguration("expected_count")

    return LaunchDescription([
        DeclareLaunchArgument(
            "image_dir", default_value="data/images/photos",
            description="测试图片目录。相对路径以运行 launch 时的当前目录为准"),
        DeclareLaunchArgument(
            "publish_rate", default_value="2.0",
            description="每秒发布多少条消息"),
        DeclareLaunchArgument(
            "use_fixed_test_data", default_value="false",
            description="true = 发写死的假坐标（只用于先验证通信）"),
        DeclareLaunchArgument(
            "expected_count", default_value="10",
            description="订阅端收够多少条就打印一次验证通过"),

        Node(
            package="cquick_vision",
            executable="target_publisher",
            name="target_publisher",
            output="screen",
            parameters=[{
                "image_dir": image_dir,
                "publish_rate": ParameterValue(publish_rate, value_type=float),
                "use_fixed_test_data": ParameterValue(use_fixed, value_type=bool),
                "output_dir": os.path.join("output", "ros2"),
            }],
        ),
        Node(
            package="cquick_vision",
            executable="target_subscriber",
            name="target_subscriber",
            output="screen",
            parameters=[{
                "expected_count": ParameterValue(expected, value_type=int),
            }],
        ),
    ])
