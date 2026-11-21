"""一条命令拉起定位节点 + 订阅打印节点。

用法：
  ros2 launch vision_comm localize.launch.py
  ros2 launch vision_comm localize.launch.py source:=0 conf:=0.25
  ros2 launch vision_comm localize.launch.py source:=/home/me/test.mp4 show_echo:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source = LaunchConfiguration('source')
    conf = LaunchConfiguration('conf')
    r = LaunchConfiguration('r')
    init_side = LaunchConfiguration('init_side')
    show_echo = LaunchConfiguration('show_echo')

    args = [
        DeclareLaunchArgument('source', default_value='1',
                              description='相机索引或视频路径（Linux 一般是 0）'),
        DeclareLaunchArgument('conf', default_value='0.20', description='标志检测置信度阈值'),
        DeclareLaunchArgument('r', default_value='0.75', description='中圈半径（米）'),
        DeclareLaunchArgument('init_side', default_value='-1',
                              description='首帧朝向先验：-1 球门对面 / +1 球门这侧 / 0 不指定'),
        DeclareLaunchArgument('show_echo', default_value='true',
                              description='是否同时启动订阅打印节点'),
    ]

    localize = Node(
        package='vision_comm',
        executable='localize',
        name='field_localizer',
        output='screen',
        parameters=[{
            'source': source,
            'conf': conf,
            'r': r,
            'init_side': init_side,
            'publish_ball': True,
            'conf_ball': 0.30,
            'min_area': 300,
        }],
    )

    echo = Node(
        package='vision_comm',
        executable='sub',
        name='pose_listener',
        output='screen',
        condition=IfCondition(show_echo),
    )

    return LaunchDescription(args + [localize, echo])
