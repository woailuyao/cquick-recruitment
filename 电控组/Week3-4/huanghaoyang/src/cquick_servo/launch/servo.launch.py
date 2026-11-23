"""一条命令把驱动节点和参数全部拉起来。

三条硬性要求都在这个文件里：
  1. 参数路径从包里查，不许写相对路径（相对路径的基准是启动时的 cwd，
     换个目录、换台机器就找不到，报错还只说 file not found）
  2. port 能被命令行覆盖：ros2 launch ... port:=/dev/ttyUSB1
  3. 客户端默认不起，加 demo:=true 才起
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 查出本包在 install 里的 share 目录，再拼 config 路径
    pkg_share = get_package_share_directory('cquick_servo')
    params_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    port = LaunchConfiguration('port')
    demo = LaunchConfiguration('demo')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/cq_servo',
                              description='串口设备路径'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='是否同时启动客户端'),

        Node(
            package='cquick_servo',
            executable='servo_driver',
            name='servo_driver',
            # 先加载 yaml，再用命令行参数覆盖 port。
            # 顺序不能反，反了 yaml 里的值会盖掉命令行
            parameters=[params_file, {'port': port}],
            output='screen',
        ),

        Node(
            package='cquick_servo',
            executable='move_client',
            name='move_client',
            condition=IfCondition(demo),
            output='screen',
        ),
    ])
