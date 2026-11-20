#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""servo.launch.py —— 一条命令把整套跑起来。

    ros2 launch cquick_servo servo.launch.py                     # 只起驱动
    ros2 launch cquick_servo servo.launch.py demo:=true          # 驱动 + 客户端
    ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1  # 换串口

三件事都做到了：
  1. 参数从包里的 yaml 读，路径用 FindPackageShare 查出来，不写相对路径；
  2. launch 参数能覆盖 yaml（用 ParameterValue(..., value_type=...)，省了 value_type
     会被当数字解析，/dev/ttyUSB1 直接报错或静默失效）；
  3. 客户端 / 播放器默认不起，加 demo:=true / player:=true 才起。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare('cquick_servo')
    params_file = PathJoinSubstitution([share, 'config', 'servo_params.yaml'])
    default_action = PathJoinSubstitution(
        [share, 'config', 'actions', 'stand_up.yaml'])

    arguments = [
        DeclareLaunchArgument('port', default_value='/dev/cq_servo',
                              description='串口设备名（udev 固定名；没装规则用 /dev/ttyUSB0）'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='波特率'),
        # ⚠️ 这两个默认值必须和 config/servo_params.yaml 保持一致，
        #    因为 launch 参数优先于 yaml，写错会把 yaml 的值悄悄盖掉。
        #    实测行程：下限 1050（硬件在 ~1007 到顶）、上限 2100。
        DeclareLaunchArgument('pwm_min', default_value='1050',
                              description='安全下限（覆盖 yaml）'),
        DeclareLaunchArgument('pwm_max', default_value='2100',
                              description='安全上限（覆盖 yaml）'),
        DeclareLaunchArgument('startup_reset', default_value='true',
                              description='上电是否全部回中位'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='true = 顺带起 move_client'),
        DeclareLaunchArgument('player', default_value='false',
                              description='true = 顺带起 sequence_player'),
        DeclareLaunchArgument('action_file', default_value=default_action,
                              description='要播放的动作 yaml 绝对路径'),
        DeclareLaunchArgument('loop_times', default_value='1',
                              description='动作组播放遍数，0 = 一直播'),
    ]

    driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'pwm_min': ParameterValue(LaunchConfiguration('pwm_min'), value_type=int),
                'pwm_max': ParameterValue(LaunchConfiguration('pwm_max'), value_type=int),
                'startup_reset': ParameterValue(LaunchConfiguration('startup_reset'),
                                                value_type=bool),
            },
        ],
    )

    demo_client = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('demo')),
        parameters=[params_file],
    )

    player = Node(
        package='cquick_servo',
        executable='sequence_player',
        name='sequence_player',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('player')),
        parameters=[
            params_file,
            {
                'action_file': ParameterValue(LaunchConfiguration('action_file'),
                                              value_type=str),
                'loop_times': ParameterValue(LaunchConfiguration('loop_times'),
                                             value_type=int),
            },
        ],
    )

    return LaunchDescription(arguments + [driver, demo_client, player])
