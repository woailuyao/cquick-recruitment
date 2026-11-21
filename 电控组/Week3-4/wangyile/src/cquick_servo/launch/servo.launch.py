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
    default_yaml = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='串口设备路径')
    demo_arg = DeclareLaunchArgument('demo', default_value='false', description='是否启动演示客户端')

    driver_node = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[
            default_yaml,
            {'port': ParameterValue(LaunchConfiguration('port'), value_type=str)}
        ]
    )

    client_node = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        condition=IfCondition(LaunchConfiguration('demo'))
    )

    return LaunchDescription([
        port_arg,
        demo_arg,
        driver_node,
        client_node
    ])