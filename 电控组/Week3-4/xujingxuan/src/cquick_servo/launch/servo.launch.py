"""一键启动：驱动节点 + （可选）客户端。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 参数文件路径必须从包里查，不能用相对路径
    params_file = os.path.join(
        get_package_share_directory('cquick_servo'), 'config', 'servo_params.yaml')

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0', description='串口设备名')
    baudrate_arg = DeclareLaunchArgument(
        'baudrate', default_value='115200', description='波特率')
    demo_arg = DeclareLaunchArgument(
        'demo', default_value='false', description='true 时同时启动客户端')

    driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',                      # 必须和 yaml 顶层 key 一致
        output='screen',
        parameters=[
            params_file,
            {
                # value_type 不能省，省了传进来就是字符串，整数参数直接报类型错
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
            },
        ],
    )

    client = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        arguments=['--id', '0', '--pwm', '1700', '--duration', '1000'],
        condition=IfCondition(LaunchConfiguration('demo')),
    )

    return LaunchDescription([port_arg, baudrate_arg, demo_arg, driver, client])
