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
    default_params = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    port = LaunchConfiguration('port')
    params_file = LaunchConfiguration('params_file')
    demo = LaunchConfiguration('demo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/cq_servo',
            description='串口设备名'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='参数文件绝对路径'),
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='是否同时启动交互客户端'),

        Node(
            package='cquick_servo',
            executable='servo_driver',
            name='servo_driver',
            output='screen',
            parameters=[
                params_file,
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
