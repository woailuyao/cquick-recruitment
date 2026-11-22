import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='auto')
    demo_arg = DeclareLaunchArgument('demo', default_value='false')

    config = os.path.join(
        get_package_share_directory('cquick_servo'),
        'config',
        'servo_params.yaml',
    )

    servo_driver_node = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        parameters=[
            config,
            {'port': ParameterValue(LaunchConfiguration('port'), value_type=str)},
        ],
        output='screen',
    )

    move_client_node = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('demo'), "' == 'true'"])
        ),
    )

    return LaunchDescription([
        port_arg,
        demo_arg,
        servo_driver_node,
        move_client_node,
    ])
