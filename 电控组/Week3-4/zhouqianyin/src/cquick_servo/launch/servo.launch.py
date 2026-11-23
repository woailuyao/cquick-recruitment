#!/usr/bin/env python3
"""一条命令起全套：ros2 launch cquick_servo servo.launch.py

常用：
    ros2 launch cquick_servo servo.launch.py                  # 用 yaml 默认值
    ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
    ros2 launch cquick_servo servo.launch.py demo:=true       # 额外起客户端演示
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'cquick_servo'


def launch_setup(context, *args, **kwargs):
    """OpaqueFunction：需要读 launch 参数的实际值来拼 parameters，必须用回调式"""
    # 用 share 目录的绝对路径，不能用源码目录相对路径（install 后源码目录不存在）
    share = get_package_share_directory(PACKAGE_NAME)
    params_file = LaunchConfiguration('params_file').perform(context).strip()
    if not params_file:
        params_file = os.path.join(share, 'config', 'servo_params.yaml')

    parameters = [params_file]

    # 命令行参数非空才覆盖 yaml，保持"yaml 为主、命令行临时改"的语义
    overrides = {}
    port = LaunchConfiguration('port').perform(context).strip()
    if port:
        overrides['port'] = port
    baudrate = LaunchConfiguration('baudrate').perform(context).strip()
    if baudrate:
        overrides['baudrate'] = int(baudrate)
    pwm_min = LaunchConfiguration('pwm_min').perform(context).strip()
    if pwm_min:
        overrides['pwm_min'] = int(pwm_min)
    pwm_max = LaunchConfiguration('pwm_max').perform(context).strip()
    if pwm_max:
        overrides['pwm_max'] = int(pwm_max)
    if overrides:
        parameters.append(overrides)    # 列表靠后的优先级更高

    driver = Node(package=PACKAGE_NAME, executable='servo_driver', name='servo_driver',
                  output='screen', emulate_tty=True, parameters=parameters)

    # demo 默认 false：正常使用时只起驱动，不自动乱动舵机
    client = Node(package=PACKAGE_NAME, executable='move_client', name='servo_client',
                  output='screen', emulate_tty=True,
                  arguments=['--id', LaunchConfiguration('servo_id'),
                             '--sweep', LaunchConfiguration('demo_sweep'),
                             '--duration', LaunchConfiguration('demo_duration')],
                  condition=IfCondition(LaunchConfiguration('demo')))
    return [driver, client]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='',
                              description='覆盖 yaml 串口设备名，如 /dev/ttyUSB1'),
        DeclareLaunchArgument('baudrate', default_value='', description='覆盖波特率'),
        DeclareLaunchArgument('pwm_min', default_value='', description='覆盖安全下限'),
        DeclareLaunchArgument('pwm_max', default_value='', description='覆盖安全上限'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='自定义参数文件绝对路径'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='true 时额外启动客户端演示'),
        DeclareLaunchArgument('servo_id', default_value='1', description='演示舵机 ID'),
        DeclareLaunchArgument('demo_sweep', default_value='1200,1800,1500',
                              description='演示动作序列'),
        DeclareLaunchArgument('demo_duration', default_value='1200',
                              description='演示单步时长 ms'),
        OpaqueFunction(function=launch_setup),
    ])
