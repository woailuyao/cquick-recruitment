from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    baud_arg = DeclareLaunchArgument('baudrate', default_value='115200')

    servo_node = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baudrate': LaunchConfiguration('baudrate')},
        ],
        output='screen'
    )
    return LaunchDescription([port_arg, baud_arg, servo_node])

