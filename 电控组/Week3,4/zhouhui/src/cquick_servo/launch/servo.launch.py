"""一键启动：驱动节点 + 可选客户端 + 可选动作组。

常用：
  ros2 launch cquick_servo servo.launch.py                      # 只起驱动，上电自动回中位
  ros2 launch cquick_servo servo.launch.py reset:=false         # 只上扭力，不产生位移
  ros2 launch cquick_servo servo.launch.py demo:=true           # 顺带起键盘客户端
  ros2 launch cquick_servo servo.launch.py play:=true           # 起驱动 + 播 stand_up.yaml
  ros2 launch cquick_servo servo.launch.py play:=true actions:=wave.yaml repeat:=3
  ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1   # 换端口（覆盖 yaml）

三个硬要求对应：参数从 yaml 读（路径用 FindPackageShare 查，不写相对路径）；
launch 参数能覆盖 yaml（port 用 value_type=str，否则 /dev/ttyUSB1 会被当数字）；
客户端默认不起，demo:=true 才起。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# 与 servo_driver.py 里的 STARTUP_RESET_MS 对应：动作组要等上电回中位走完再发第一个 goal
STARTUP_RESET_S = 4.0


def generate_launch_description():
    args = [
        DeclareLaunchArgument("port", default_value="/dev/cq_servo",
                              description="串口设备名，默认值来自 udev 固定名"),
        DeclareLaunchArgument("reset", default_value="true",
                              description="上电是否全部回 1500 中立位"),
        DeclareLaunchArgument("demo", default_value="false",
                              description="是否起键盘客户端"),
        DeclareLaunchArgument("play", default_value="false",
                              description="是否起动作组播放器"),
        DeclareLaunchArgument("actions", default_value="stand_up.yaml",
                              description="config/actions/ 下的动作文件名"),
        DeclareLaunchArgument("repeat", default_value="1",
                              description="动作组重复次数"),
    ]

    share = FindPackageShare(package="cquick_servo")
    params_file = PathJoinSubstitution([share, "config", "servo_params.yaml"])

    driver = Node(
        package="cquick_servo",
        executable="servo_driver",
        name="servo_driver",
        parameters=[
            params_file,
            {
                # value_type 不能省，省了会被当表达式解析
                "port": ParameterValue(LaunchConfiguration("port"), value_type=str),
                "startup_reset": ParameterValue(LaunchConfiguration("reset"), value_type=bool),
            },
        ],
        output="screen",
    )

    client = Node(
        package="cquick_servo",
        executable="move_client",
        name="move_client",
        output="screen",
        condition=IfCondition(LaunchConfiguration("demo")),
    )

    player = TimerAction(
        period=STARTUP_RESET_S + 0.5,
        actions=[Node(
            package="cquick_servo",
            executable="sequence_player",
            name="sequence_player",
            parameters=[{
                "actions": ParameterValue(LaunchConfiguration("actions"), value_type=str),
                "repeat": ParameterValue(LaunchConfiguration("repeat"), value_type=int),
                "show_feedback": False,
            }],
            output="screen",
            condition=IfCondition(LaunchConfiguration("play")),
        )],
    )

    return LaunchDescription(args + [driver, client, player])
