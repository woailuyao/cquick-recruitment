from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('cquick_servo')
    default_yaml = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial port'),
        DeclareLaunchArgument('demo', default_value='false', description='Start demo client'),
        DeclareLaunchArgument('params_file', default_value=default_yaml, description='Params file'),

        Node(
            package='cquick_servo',
            executable='servo_driver',
            name='servo_driver',
            parameters=[LaunchConfiguration('params_file'), {'port': LaunchConfiguration('port')}],
            output='screen'
        ),

        Node(
            package='cquick_servo',
            executable='move_client',
            name='move_client',
            condition=IfCondition(LaunchConfiguration('demo')),
            output='screen'
        )
    ])
