# -*- coding: utf-8 -*-
"""servo.launch.py —— 一条命令把驱动节点、参数、可选客户端/播放器全起来。

    ros2 launch cquick_servo servo.launch.py
    ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
    ros2 launch cquick_servo servo.launch.py demo:=true
    ros2 launch cquick_servo servo.launch.py sequence:=true action_file:=wave.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('cquick_servo')

    # 用 PathJoinSubstitution 在运行期拼出 install/share 下的绝对路径，
    # 不要写相对路径（换一台机器必挂）
    default_params = PathJoinSubstitution([pkg_share, 'config', 'servo_params.yaml'])

    port = LaunchConfiguration('port')
    demo = LaunchConfiguration('demo')
    sequence = LaunchConfiguration('sequence')
    action_file = LaunchConfiguration('action_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='串口设备（udev 固定名；没有 udev 规则就传 port:=/dev/ttyUSB0）'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='参数 yaml 的绝对路径'),
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='true 时额外启动 move_client（交互发 goal）'),
        DeclareLaunchArgument(
            'sequence', default_value='false',
            description='true 时启动动作组播放器'),
        DeclareLaunchArgument(
            'action_file', default_value='stand_up.yaml',
            description='播放哪个动作文件（share/config/actions 下的名字或绝对路径）'),
        DeclareLaunchArgument(
            'require_prad', default_value='false',
            description='默认 false：读不到回显时用最后下发的 PWM 当结果；true=严格模式'),

        # ① 驱动节点：先加载 yaml，再用 launch 参数覆盖
        Node(
            package='cquick_servo',
            executable='servo_driver',
            name='servo_driver',          # 必须和 yaml 顶层 key 一致
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'port': ParameterValue(port, value_type=str),
                    'require_prad': ParameterValue(
                        LaunchConfiguration('require_prad'), value_type=bool),
                },
            ],
        ),

        # ② 客户端：默认不起
        Node(
            package='cquick_servo',
            executable='move_client',
            name='move_client',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(demo),
        ),

        # ③ 动作组播放器：默认不起，跑完自己退出
        Node(
            package='cquick_servo',
            executable='sequence_player',
            name='sequence_player',
            output='screen',
            emulate_tty=True,
            parameters=[{'action_file': ParameterValue(action_file, value_type=str)}],
            condition=IfCondition(sequence),
        ),
    ])
