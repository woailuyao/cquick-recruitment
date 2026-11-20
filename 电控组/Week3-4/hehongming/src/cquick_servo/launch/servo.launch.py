import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('cquick_servo')
    default_yaml = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    # 声明 launch 参数
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    demo_arg = DeclareLaunchArgument('demo', default_value='false') # 条件启动

    driver_node = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        parameters=[default_yaml, {'port': LaunchConfiguration('port')}]
    )

    client_node = Node(
        package='cquick_servo',
        executable='move_client',
        condition=IfCondition(LaunchConfiguration('demo'))
    )

    return LaunchDescription([
        port_arg,
        demo_arg,
        driver_node,
        client_node
    ])