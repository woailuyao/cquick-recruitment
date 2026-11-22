import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('cquick_servo')
    yaml_path = os.path.join(pkg_dir, 'config', 'servo_params.yaml')
    port = LaunchConfiguration('port').perform(context)

    return [
        Node(
            package='cquick_servo',
            executable='servo_driver',
            name='servo_driver',
            output='screen',
            parameters=[
                yaml_path,
                {'port': ParameterValue(port, value_type=str)},
            ],
        ),
        Node(
            package='cquick_servo',
            executable='move_client',
            name='move_client',
            output='screen',
            condition=IfCondition(LaunchConfiguration('demo')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='串口设备名，可覆盖 yaml 中的默认值'),
        DeclareLaunchArgument(
            'demo',
            default_value='false',
            description='是否启动演示客户端节点'),
        OpaqueFunction(function=launch_setup),
    ])