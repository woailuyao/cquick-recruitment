# -*- coding: utf-8 -*-
"""一条命令启动：驱动 + yaml 参数 + 可选客户端。

    ros2 launch cquick_servo servo.launch.py
    ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
    ros2 launch cquick_servo servo.launch.py demo:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'cquick_servo'


def _launch_setup(context, *args, **kwargs):
    # 用 FindPackageShare 查包内绝对路径，绝不写相对路径（换机器也能跑）
    pkg_share = FindPackageShare(PACKAGE)
    params_file = PathJoinSubstitution([pkg_share, 'config', 'servo_params.yaml'])

    port = LaunchConfiguration('port').perform(context).strip()
    demo = LaunchConfiguration('demo').perform(context).strip().lower() in (
        '1', 'true', 'yes', 'on'
    )

    # 先加载 yaml，再用 launch 参数覆盖（后加的字典优先级更高）
    parameters = [params_file]
    if port:
        # 注意传纯字符串，不套 ParameterValue —— 套了要写 value_type=str，
        # 否则 /dev/ttyUSB1 会被当成数字解析而报错（FAQ Q6 那个坑）
        parameters.append({'port': port})

    nodes = [
        Node(
            package=PACKAGE,
            executable='servo_driver',
            name='servo_driver',
            output='screen',
            emulate_tty=True,
            parameters=parameters,
        )
    ]

    if demo:
        # 有条件的辅助节点：默认不起，demo:=true 才起
        nodes.append(
            Node(
                package=PACKAGE,
                executable='move_client',
                name='move_client',
                output='screen',
                emulate_tty=True,
                arguments=['1', '1700', '1000'],   # 演示一条：1 号去 1700，用 1 秒
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='',
            description='覆盖 yaml 里的 port；留空表示用 yaml 的值',
        ),
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='true 时额外启动 Action 客户端演示一条动作',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
