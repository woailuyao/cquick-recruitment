from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('cquick_servo')
    params = PathJoinSubstitution([share, 'config', 'servo_params.yaml'])

    port = LaunchConfiguration('port')
    demo = LaunchConfiguration('demo')
    reset = LaunchConfiguration('startup_reset')

    driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[
            params,
            {'port': ParameterValue(port, value_type=str)},
            {'startup_reset': ParameterValue(reset, value_type=bool)},
        ],
    )

    client = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        condition=IfCondition(demo),
        parameters=[],
        arguments=['--id', '1', '--pwm', '1700', '--duration', '1200'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='串口设备，覆盖 yaml'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='true 时额外启动客户端节点'),
        DeclareLaunchArgument('startup_reset', default_value='true',
                              description='启动时全部回中位'),
        driver,
        client,
    ])
