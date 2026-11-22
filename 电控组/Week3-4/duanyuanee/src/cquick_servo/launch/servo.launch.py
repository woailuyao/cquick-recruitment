import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('cquick_servo')
    params_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')
    return LaunchDescription([
        # ===== 启动参数 =====
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='串口设备名'
        ),
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='true=启动客户端演示节点'
        ),

        # ===== 驱动节点（必起）=====
        Node(
            package='cquick_servo',
            executable='servo_driver',
            name='servo_driver',
            output='screen',
            parameters=[
                params_file,
                {'port': LaunchConfiguration('port')},
            ]
        ),

        # ===== 客户端（可选，demo:=true才起）=====
        Node(
            package='cquick_servo',
            executable='move_client',
            name='move_client',
            output='screen',
            condition=IfCondition(LaunchConfiguration('demo'))
        ),
    ])
