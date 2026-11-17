from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('cquick_servo'), 'config', 'servo_params.yaml'])

    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                                     description='串口设备，可覆盖 yaml')
    demo_arg = DeclareLaunchArgument('demo', default_value='false',
                                     description='是否同时启动客户端')

    driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[params_file, {'port': LaunchConfiguration('port')}],
    )

    client = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        condition=IfCondition(LaunchConfiguration('demo')),
    )

    return LaunchDescription([port_arg, demo_arg, driver, client])
