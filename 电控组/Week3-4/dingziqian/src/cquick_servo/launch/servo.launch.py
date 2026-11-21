import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory("cquick_servo")
    params_file = os.path.join(share_dir, "config", "servo_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/cq_servo"),
        DeclareLaunchArgument("demo", default_value="false"),
        DeclareLaunchArgument("demo_id", default_value="1"),
        DeclareLaunchArgument("demo_pwm", default_value="1500"),
        Node(
            package="cquick_servo",
            executable="servo_driver",
            name="servo_driver",
            output="screen",
            parameters=[
                params_file,
                {"port": ParameterValue(
                    LaunchConfiguration("port"), value_type=str
                )},
            ],
        ),
        Node(
            package="cquick_servo",
            executable="move_client",
            name="move_client",
            output="screen",
            condition=IfCondition(LaunchConfiguration("demo")),
            arguments=[
                "--id", LaunchConfiguration("demo_id"),
                "--pwm", LaunchConfiguration("demo_pwm"),
            ],
        ),
    ])
