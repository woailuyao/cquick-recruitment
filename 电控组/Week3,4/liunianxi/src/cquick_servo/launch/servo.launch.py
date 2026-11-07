"""一键启动：驱动 +（可选）动作组演示。

launch 参数（细则 2.5 要求）：
  params_file   参数文件路径，默认包内 config/servo_params.yaml
  port          覆盖 yaml 里的串口设备名（空 = 用 yaml 里的值）
  demo          true 时延时 3 秒自动播一遍动作组（条件启动）
  actions_file  demo 用哪个动作组 yaml
  log_level     DEBUG / INFO / WARN ...

用法：
  ros2 launch cquick_servo servo.launch.py
  ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
  ros2 launch cquick_servo servo.launch.py demo:=true actions_file:=/abs/path/wave.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _driver_node(context):
    """用 OpaqueFunction 才能「有 port 才覆盖」这种条件参数。"""
    params = [LaunchConfiguration("params_file")]
    port = LaunchConfiguration("port").perform(context)
    overrides = {}
    if port.strip():
        overrides["port"] = port.strip()
    if overrides:
        params.append(overrides)
    return [
        Node(
            package="cquick_servo",
            executable="servo_driver",
            name="servo_driver",
            output="screen",
            parameters=params,
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("cquick_servo"), "config", "servo_params.yaml"]
    )
    default_actions = PathJoinSubstitution(
        [FindPackageShare("cquick_servo"), "config", "actions", "wave.yaml"]
    )

    demo_player = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="cquick_servo",
                executable="sequence_player",
                name="demo_player",
                output="screen",
                parameters=[
                    {"actions_file": LaunchConfiguration("actions_file")},
                    {"dry_run": False},
                ],
            )
        ],
        condition=IfCondition(LaunchConfiguration("demo")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=default_params, description="参数文件"
            ),
            DeclareLaunchArgument("port", default_value="", description="覆盖串口设备名"),
            DeclareLaunchArgument(
                "demo", default_value="false", description="true 则自动播一遍动作组"
            ),
            DeclareLaunchArgument(
                "actions_file", default_value=default_actions, description="动作组 yaml"
            ),
            DeclareLaunchArgument(
                "log_level", default_value="INFO", description="日志级别"
            ),
            LogInfo(msg=["启动 servo_driver，参数文件由 launch arg 决定"]),
            OpaqueFunction(function=_driver_node),
            demo_player,
        ]
    )
