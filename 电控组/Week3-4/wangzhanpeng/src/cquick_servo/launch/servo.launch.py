#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('cquick_servo')
    config_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    port = LaunchConfiguration('port')
    demo = LaunchConfiguration('demo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/cq_servo',
            description='串口设备，例如 /dev/ttyUSB1 或 /dev/cq_servo'),
        DeclareLaunchArgument(
            'demo',
            default_value='false',
            description='true 时额外启动 move_client'),

        Node(
            package='cquick_servo',
            executable='servo_driver',
            name='servo_driver',
            output='screen',
            parameters=[
                config_file,
                {'port': ParameterValue(port, value_type=str)},
            ],
        ),
        Node(
            package='cquick_servo',
            executable='move_client',
            name='move_client',
            output='screen',
            condition=IfCondition(demo),
        ),
    ])
