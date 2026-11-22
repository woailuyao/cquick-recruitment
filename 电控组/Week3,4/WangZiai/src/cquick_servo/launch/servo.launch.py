"""一条命令起全部：驱动 + （可选）演示客户端。

    ros2 launch cquick_servo servo.launch.py                    # 硬件模式（默认 /dev/cq_servo）
    ros2 launch cquick_servo servo.launch.py port:=mock         # 无硬件
    ros2 launch cquick_servo servo.launch.py demo:=true port:=mock
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

# ros拓展内容
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 路径拼装
    pkg_share = FindPackageShare('cquick_servo')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'servo_params.yaml'])

    # 参数与条件
    port = LaunchConfiguration('port')
    demo = LaunchConfiguration('demo')
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/cq_servo',
            description='串口设备；未装 udev 用 /dev/ttyUSB0，无硬件用 mock'
            ),
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='true 时额外启动 move_client 演示客户端'
            ),

        # 驱动节点
        Node(

            # 定位
            package='cquick_servo',
            executable='servo_driver',

            # 强制节点名
            name='servo_driver',

            # 日志打印至终端
            output='screen',

            # 后覆盖前的参数并指定类型
            parameters=[
                params_file,                                  # 先读 yaml
                {'port': ParameterValue(port, value_type=str)},  # 再用 launch 参数覆盖
            ],
        ),

        # 演示客户端
        Node(
            package='cquick_servo',
            executable='move_client',
            name='move_client',
            output='screen',
            condition=IfCondition(demo),
        ),
    ])
