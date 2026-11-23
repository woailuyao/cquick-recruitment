"""一键启动：驱动节点 + 可选客户端。

关键点：
  1. 参数从包内 config/servo_params.yaml 读取，路径用 get_package_share_directory 查，
     不能用相对路径（在别人机器上必挂）
  2. port / demo 等可以用 launch 参数覆盖：
        ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
  3. 客户端默认不启动，demo:=true 才起
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
    pkg_share = get_package_share_directory('cquick_servo')
    params_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='串口设备名，例如 /dev/ttyUSB0 或 /dev/cq_servo')

    sim_arg = DeclareLaunchArgument(
        'sim', default_value='false',
        description='无硬件时置 true，用模拟数据验证 Action / 参数 / launch 逻辑')

    demo_arg = DeclareLaunchArgument(
        'demo', default_value='false',
        description='是否顺带启动示例客户端')

    driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[
            params_file,
            {
                # 注意 value_type=str，不然 /dev/ttyUSB1 会被当数字解析
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'sim': ParameterValue(LaunchConfiguration('sim'), value_type=bool),
            },
        ],
    )

    client = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        arguments=['--demo'],
        condition=IfCondition(LaunchConfiguration('demo')),
    )

    return LaunchDescription([port_arg, sim_arg, demo_arg, driver, client])
