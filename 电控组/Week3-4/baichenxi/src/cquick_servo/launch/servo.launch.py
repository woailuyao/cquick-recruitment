import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('cquick_servo')
    param_file = os.path.join(pkg_dir, 'config', 'servo_params.yaml')
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    servo_node = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        parameters=[param_file, {'port': LaunchConfiguration('port')}]
    )
    return LaunchDescription([port_arg, servo_node])
