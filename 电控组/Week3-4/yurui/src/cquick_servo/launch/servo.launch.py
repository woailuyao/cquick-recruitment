import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 从安装目录里找 yaml，不许写死相对路径
    config = os.path.join(
        get_package_share_directory("cquick_servo"),
        "config",
        "servo_params.yaml",
    )
    return LaunchDescription([
        Node(
            package="cquick_servo",
            executable="servo_driver",
            name="servo_driver",
            parameters=[config],
        ),
    ])

