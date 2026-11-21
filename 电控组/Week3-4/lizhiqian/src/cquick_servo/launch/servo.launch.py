import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('cquick_servo')
    params_file = os.path.join(pkg_dir, 'config', 'servo_params.yaml')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='串口设备名',
    )
    demo_arg = DeclareLaunchArgument(
        'demo',
        default_value='false',
        description='是否同时启动 move_client',
    )

    port_value = LaunchConfiguration('port')
    demo_value = LaunchConfiguration('demo')

    driver_node = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        parameters=[
            params_file,
            {'port': ParameterValue(port_value, value_type=str)},
        ],
        output='screen',
    )

    client_node = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        parameters=[{'id': 1, 'pwm': 1500, 'duration_ms': 1000}],
        output='screen',
        condition=IfCondition(demo_value),
    )

    return LaunchDescription([
        port_arg,
        demo_arg,
        driver_node,
        client_node,
    ])
