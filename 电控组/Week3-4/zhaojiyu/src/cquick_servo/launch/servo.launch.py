#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cquick_servo 一键启动"""
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
    params_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    declared_args = [
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='串口设备，例如 /dev/ttyUSB1 或 /dev/cq_servo'),
        DeclareLaunchArgument(
            'baudrate', default_value='115200'),
        DeclareLaunchArgument(
            'pwm_max', default_value='2100'),
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='true 时同时启动 move_client'),
    ]

    driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[
            params_file,
            {
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'pwm_max': ParameterValue(LaunchConfiguration('pwm_max'), value_type=int),
            },
        ],
    )

    client = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        condition=IfCondition(LaunchConfiguration('demo')),
    )

    return LaunchDescription(declared_args + [driver, client])
