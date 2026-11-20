"""一条命令启动驱动节点（可选带客户端 demo）。

    ros2 launch cquick_servo servo.launch.py
    ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
    ros2 launch cquick_servo servo.launch.py demo:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _build_nodes(context):
    share = get_package_share_directory('cquick_servo')
    params_file = os.path.join(share, 'config', 'servo_params.yaml')

    parameters = [params_file]

    # launch 参数能覆盖 yaml：只有显式给了 port 才加进去
    port = LaunchConfiguration('port').perform(context).strip()
    if port:
        parameters.append({'port': ParameterValue(port, value_type=str)})

    driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=parameters,
    )

    client = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        condition=IfCondition(LaunchConfiguration('demo')),
    )

    return [driver, client]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='',
            description='覆盖 yaml 里的串口设备名，例如 /dev/ttyUSB1'),
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='true 时额外启动 move_client'),
        OpaqueFunction(function=_build_nodes),
    ])
