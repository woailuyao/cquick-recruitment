from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    image_path = LaunchConfiguration("image_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_path",
                default_value="test_images/clear.png",
                description="发布节点读取的图片路径",
            ),
            Node(
                package="cquick_color_vision",
                executable="publish_target",
                name="target_position_publisher",
                parameters=[{"image_path": image_path}],
                output="screen",
            ),
            Node(
                package="cquick_color_vision",
                executable="subscribe_target",
                name="target_position_subscriber",
                output="screen",
            ),
        ]
    )
