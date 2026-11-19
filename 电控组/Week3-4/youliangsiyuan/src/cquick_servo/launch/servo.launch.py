#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一条命令把整套系统拉起来（细则 2.5，7 分）。

    ros2 launch cquick_servo servo.launch.py

三件必须做到的事：
  1. 参数从 yaml 读 —— 路径用 get_package_share_directory 查出来，**不许写相对路径**
     （相对路径在别人机器上必挂）；
  2. launch 参数能覆盖 yaml —— `ros2 launch ... port:=/dev/ttyUSB1` 真的换端口；
  3. 有条件启动 —— 客户端默认不起，加 demo:=true 才起。

⭐ 最容易踩的坑：launch/ 和 config/ 默认**不会**被装进 install 里，
   必须在 setup.py 的 data_files 里显式加。改完必须重新 colcon build，
   不然 install 里还是旧文件（细则 FAQ Q4）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ① 用包查出来的绝对路径，不用相对路径
    share_dir = get_package_share_directory('cquick_servo')
    default_params_file = os.path.join(share_dir, 'config', 'servo_params.yaml')

    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    demo = LaunchConfiguration('demo')
    action_file = LaunchConfiguration('action_file')
    replay_mode = LaunchConfiguration('replay_mode')

    declare_arguments = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='参数 yaml 的绝对路径（默认用包里装好的那份）'),
        # ② 这三个是"覆盖 yaml"用的。命令行写 port:=/dev/ttyUSB1 就能真的换端口
        DeclareLaunchArgument(
            'port', default_value='/dev/cq_servo',
            description='串口设备名；默认走 udev 固定名 /dev/cq_servo，没有 udev 规则时用 port:=/dev/ttyUSB0 覆盖'),
        DeclareLaunchArgument(
            'baudrate', default_value='115200',
            description='波特率；覆盖 yaml 里的 baudrate'),
        DeclareLaunchArgument(
            'replay_mode', default_value='false',
            description='true = 只认 /servo/command（ros2 bag play 回放用）'),
        # ③ 条件启动开关
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='true = 顺带把动作组播放器也起起来'),
        DeclareLaunchArgument(
            'action_file', default_value='squat_in_place.yaml',
            description='demo:=true 时播放哪个动作 yaml（config/actions/ 下的文件名）'),
    ]

    servo_driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        # ⚠️ 这个名字必须和 servo_params.yaml 最外层那个 key 一模一样，否则参数静默不加载
        name='servo_driver',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,          # 先加载 yaml
            {                     # 再用 launch 参数覆盖（列表后面的赢）
                # value_type 不能省！不写会被当数字解析，"/dev/ttyUSB1" 直接报错或忽略
                'port': ParameterValue(port, value_type=str),
                'baudrate': ParameterValue(baudrate, value_type=int),
                'replay_mode': ParameterValue(replay_mode, value_type=bool),
            },
        ],
    )

    sequence_player = Node(
        package='cquick_servo',
        executable='sequence_player',
        name='sequence_player',
        output='screen',
        emulate_tty=True,
        # ③ 不加 demo:=true 就不起
        condition=IfCondition(demo),
        parameters=[{
            'action_file': ParameterValue(action_file, value_type=str),
        }],
    )

    return LaunchDescription(declare_arguments + [servo_driver, sequence_player])
