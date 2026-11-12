"""可选的一键启动；初次实操建议仍用两个终端分别看发布和订阅日志。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_dir', description='必填：图片目录，不递归子目录'),
        DeclareLaunchArgument('output_dir', default_value='', description='订阅记录目录；空则不写文件'),
        DeclareLaunchArgument('max_messages', default_value='0', description='订阅条数上限；0 表示持续接收'),
        Node(
            package='color_target_vision', executable='target_publisher', output='screen',
            parameters=[{
                'image_dir': ParameterValue(LaunchConfiguration('image_dir'), value_type=str),
                'publish_hz': 2.0,
            }],
        ),
        Node(
            package='color_target_vision', executable='target_subscriber', output='screen',
            parameters=[{
                'output_dir': ParameterValue(LaunchConfiguration('output_dir'), value_type=str),
                'max_messages': ParameterValue(LaunchConfiguration('max_messages'), value_type=int),
            }],
        ),
    ])
