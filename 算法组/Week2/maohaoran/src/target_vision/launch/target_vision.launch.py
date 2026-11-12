"""同时启动发布节点和订阅节点。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='target_vision',
            executable='target_publisher',
            name='target_publisher',
            output='screen',
        ),
        Node(
            package='target_vision',
            executable='target_subscriber',
            name='target_subscriber',
            output='screen',
        ),
    ])
