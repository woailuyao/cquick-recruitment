"""servo.launch.py —— 一条命令起全部

ros2 launch cquick_servo servo.launch.py                    # 只起驱动节点
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1  # 换串口
ros2 launch cquick_servo servo.launch.py demo:=true          # 顺手起客户端做正反转回中位
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('cquick_servo')
    default_params = os.path.join(share, 'config', 'servo_params.yaml')
    default_action = os.path.join(share, 'config', 'actions', 'stand_up.yaml')

    port = LaunchConfiguration('port')
    params_file = LaunchConfiguration('params_file')
    demo = LaunchConfiguration('demo')
    dry_run = LaunchConfiguration('dry_run')
    action_file = LaunchConfiguration('action_file')

    declared = [
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='串口设备名，例如 /dev/ttyUSB1'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='参数 yaml 绝对路径'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='true 时同时启动 move_client 做正反转回中位'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true 时不打开串口，只打印指令（桌面无硬件时调试用）'),
        DeclareLaunchArgument('action_file', default_value=default_action,
                              description='动作组 yaml，供 sequence_player 使用'),
    ]

    driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[
            params_file,
            {
                # value_type=str 不能省，否则 /dev/ttyUSB1 会被当数字解析
                'port': ParameterValue(port, value_type=str),
                'dry_run': ParameterValue(dry_run, value_type=bool),
            },
        ],
    )

    client = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        condition=IfCondition(demo),
        parameters=[{
            'servo_id': 1,
            'pwm_list': [1700, 1300, 1500],
            'duration_ms': 1000,
        }],
    )

    player = Node(
        package='cquick_servo',
        executable='sequence_player',
        name='sequence_player',
        output='screen',
        condition=IfCondition(LaunchConfiguration('play')),
        parameters=[{'action_file': ParameterValue(action_file, value_type=str)}],
    )

    declared.append(
        DeclareLaunchArgument('play', default_value='false',
                              description='true 时同时启动 actions 播放器')
    )

    return LaunchDescription(declared + [driver, client, player])
