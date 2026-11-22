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
    params = os.path.join(share, 'config', 'servo_params.yaml')

    port = LaunchConfiguration('port')
    demo = LaunchConfiguration('demo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='串口设备名，覆盖 yaml 里的 port'),
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='true 时额外启动演示客户端'),

        Node(
            package='cquick_servo',
            executable='servo_driver',
            name='servo_driver',
            output='screen',
            parameters=[params, {'port': ParameterValue(port, value_type=str)}],
        ),

        Node(
            package='cquick_servo',
            executable='move_client',
            name='move_client',
            output='screen',
            condition=IfCondition(demo),
            parameters=[{'id': 1, 'pwm': 1700, 'duration_ms': 1000}],
        ),
    ])
