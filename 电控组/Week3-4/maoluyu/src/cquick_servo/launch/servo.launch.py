from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'cquick_servo'
    pkg_share = get_package_share_directory(pkg_name)

    # yaml参数文件路径
    yaml_path = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    servo_node = Node(
        package=pkg_name,
        executable='servo_driver',
        name='servo_driver',
        parameters=[yaml_path],
        output='screen'
    )

    return LaunchDescription([
        servo_node
    ])

