import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 用包名查出 config 目录的绝对路径（不许写相对路径，换台机器就挂）
    config_dir = os.path.join(
        get_package_share_directory('cquick_servo'), 'config')
    params_file = os.path.join(config_dir, 'servo_params.yaml')

    # 声明 launch 参数，默认值可以被命令行覆盖
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0')
    demo_arg = DeclareLaunchArgument(
        'demo', default_value='false')   # 客户端默认不起

    # 驱动节点：参数从 yaml 加载，port 用 launch 参数覆盖
    driver_node = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',           # 节点名必须和 yaml 顶层 key 一致
        parameters=[params_file, {'port': LaunchConfiguration('port')}],
        output='screen',               # 日志直接打到终端，截图要用
    )

    # 客户端节点：只有 demo:=true 才起
    client_node = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        condition=IfCondition(LaunchConfiguration('demo')),
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        demo_arg,
        driver_node,
        client_node,
    ])
