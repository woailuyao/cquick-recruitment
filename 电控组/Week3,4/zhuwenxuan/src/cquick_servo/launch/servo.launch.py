#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cquick_servo 一键启动（第 3/4 周 · 任务 2.5）

    ros2 launch cquick_servo servo.launch.py
    ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
    ros2 launch cquick_servo servo.launch.py demo:=true demo_pwm:=1800
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('cquick_servo')
    param_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    # 参数优先级：yaml 先加载，launch 传入的再覆盖
    parameters = [param_file]
    port = LaunchConfiguration('port').perform(context)
    if port:
        parameters.append({'port': port})

    nodes = [
        Node(
            package='cquick_servo',
            executable='servo_driver',
            name='servo_driver',          # 必须和 yaml 最外层 key 一致，否则参数不加载
            output='screen',
            parameters=parameters,
        )
    ]

    if LaunchConfiguration('demo').perform(context).lower() in ('true', '1', 'yes'):
        nodes.append(Node(
            package='cquick_servo',
            executable='move_client',
            name='move_client',
            output='screen',
            arguments=[
                '--id', LaunchConfiguration('demo_id'),
                '--pwm', LaunchConfiguration('demo_pwm'),
                '--sweep',
            ],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='',
                              description='覆盖 yaml 里的串口设备，如 /dev/ttyUSB1'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='true 时额外起客户端跑一遍正反转回中位'),
        DeclareLaunchArgument('demo_id', default_value='1'),
        DeclareLaunchArgument('demo_pwm', default_value='1800'),
        OpaqueFunction(function=_launch_setup),
    ])
