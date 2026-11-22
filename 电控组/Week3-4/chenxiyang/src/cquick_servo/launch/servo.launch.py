import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('cquick_servo')
    config_file = os.path.join(pkg_dir, 'config', 'servo_params.yaml')

    servo_driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        servo_driver
    ])
