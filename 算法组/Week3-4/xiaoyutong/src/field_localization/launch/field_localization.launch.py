"""Launch 文件：启动单目场地定位节点。

用法：
    export FIELD_ROOT=$HOME/cquick-recruitment/算法组/Week3-4/xiaoyutong
    ros2 launch field_localization field_localization.launch.py
    # 指定权重
    ros2 launch field_localization field_localization.launch.py weights:=$HOME/best.pt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("weights", default_value="",
                              description="YOLO 权重路径（best.pt）"),
        DeclareLaunchArgument("detector", default_value="yolo",
                              description="classical / yolo / click"),
        DeclareLaunchArgument("publish_debug_image", default_value="true"),
        Node(
            package="field_localization",
            executable="field_localization_node.py",
            name="field_localization",
            output="screen",
            parameters=[{
                "weights": LaunchConfiguration("weights"),
                "detector": LaunchConfiguration("detector"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            }],
        ),
    ])
