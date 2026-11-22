# -*- coding: utf-8 -*-
"""
可复用 launch 文件。

默认启动 bridge_node（接收 Windows 侧的实拍定位结果）。
把 mode 设为 "synthetic" 时改为启动 pose_publisher，用于无相机联调。

  ros2 launch k1_localization_ros localization.launch.py
  ros2 launch k1_localization_ros localization.launch.py port:=5057
  ros2 launch k1_localization_ros localization.launch.py mode:=synthetic
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    port = LaunchConfiguration("port")
    rate = LaunchConfiguration("rate_hz")

    is_bridge = PythonExpression(["'", mode, "' == 'bridge'"])
    is_synth = PythonExpression(["'", mode, "' == 'synthetic'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="bridge",
            description="bridge=接收 Windows 实拍结果；synthetic=发布合成位姿用于联调"),
        DeclareLaunchArgument(
            "port", default_value="5056",
            description="bridge 模式的 TCP 监听端口"),
        DeclareLaunchArgument(
            "rate_hz", default_value="10.0",
            description="synthetic 模式的发布频率"),

        Node(
            package="k1_localization_ros",
            executable="bridge_node",
            name="k1_field_pose_relay",
            output="screen",
            parameters=[{"port": port, "host": "0.0.0.0", "qos_depth": 10}],
            condition=IfCondition(is_bridge),
        ),
        Node(
            package="k1_localization_ros",
            executable="pose_publisher",
            name="k1_synthetic_pose_publisher",
            output="screen",
            parameters=[{"rate_hz": rate}],
            condition=IfCondition(is_synth),
        ),
    ])
