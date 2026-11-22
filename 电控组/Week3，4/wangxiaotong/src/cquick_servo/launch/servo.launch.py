import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('cquick_servo')
    params_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    port = LaunchConfiguration('port')
    demo = LaunchConfiguration('demo')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/cquick_servo',
                              description='串口设备，模拟时传 /tmp/ttyCQ'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='true 时额外发一条演示指令'),

        Node(
            package='cquick_servo',
            executable='servo_driver',
            name='servo_driver',
            output='screen',
            # 先读 yaml，再用命令行覆盖 port —— 后写的优先
            parameters=[params_file, {'port': port}],
        ),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/servo/command',
                 'cquick_servo_interfaces/msg/ServoCommand',
                 '{id: 1, pwm: 1800, duration_ms: 1500}'],
            condition=IfCondition(demo),
            output='screen',
        ),
    ])
