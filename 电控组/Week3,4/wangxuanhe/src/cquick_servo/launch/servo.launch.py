"""一键启动：驱动 + 上电初始姿态 + 动作组。

  ros2 launch cquick_servo servo.launch.py                         # 上电即站姿（只回一次 1500）
  ros2 launch cquick_servo servo.launch.py reset:=false            # 只上扭力，不产生任何位移
  ros2 launch cquick_servo servo.launch.py play:=true              # 站好后播 stand_up.yaml
  ros2 launch cquick_servo servo.launch.py play:=true actions:=wave.yaml repeat:=3
  ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1      # 换端口（覆盖 yaml）

上电顺序：driver 上扭力（PULR，不位移）→ 紧接着一条 #xxxP1500T4000! 把整机压到中立位站姿，
4 秒走完，之后就一直保持，不再二次修正。这就是「上电初始状态」，不需要 launch 再做一次。

所以 home_enable 默认 false：上电姿态由 driver 的 reset 一次搞定。只有想把整机摆到某个
自定义 yaml 姿态（不是 1500）时才 home_enable:=true，此时请同时 reset:=false，否则两个
命令抢同一批关节，肉眼看到的就是上电先抖一下再走。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

RESET_DURATION_MS = 4000   # 与 servo_driver.py 的 RESET_DURATION_MS 保持一致


def _true(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ("true", "1")


def generate_launch_description():
    """生成节点描述表。"""
    port_arg = DeclareLaunchArgument("port", default_value="/dev/cq_servo")
    demo_arg = DeclareLaunchArgument("demo", default_value="false")
    reset_arg = DeclareLaunchArgument(
        "reset", default_value="true",
        description="开机是否把全体压到 1500 中立位即站姿（一次 4 s 走完，不二次修正）")
    home_arg = DeclareLaunchArgument(
        "home", default_value="home.yaml",
        description="自定义初始姿态文件名（放 config/actions/ 下，配合 home_enable:=true）")
    home_enable_arg = DeclareLaunchArgument(
        "home_enable", default_value="false",
        description="上电是否再摆一次自定义姿态，默认 false（reset 已经摆好了）")
    play_arg = DeclareLaunchArgument("play", default_value="false")
    actions_arg = DeclareLaunchArgument("actions", default_value="stand_up.yaml")
    repeat_arg = DeclareLaunchArgument("repeat", default_value="1")

    pkg_share = FindPackageShare(package="cquick_servo").find("cquick_servo")
    yaml_path = PathJoinSubstitution([pkg_share, "config", "servo_params.yaml"])

    driver_node = Node(
        package="cquick_servo",
        executable="servo_driver",
        name="servo_driver",
        parameters=[yaml_path, {
            # value_type=str 不能省，否则 /dev/ttyUSB1 会被当数字解析
            "port": ParameterValue(LaunchConfiguration("port"), value_type=str),
            "startup_reset": ParameterValue(
                LaunchConfiguration("reset"), value_type=bool),
        }],
        output="screen",
    )

    client_node = Node(
        package="cquick_servo",
        executable="move_client",
        name="servo_client",
        output="screen",
        condition=IfCondition(LaunchConfiguration("demo")),
    )

    # 动作组播放器：等 reset 的 4 s 走完再发第一个 goal，否则和上电回中被抢
    player_node = TimerAction(period=RESET_DURATION_MS / 1000.0 + 0.5, actions=[Node(
        package="cquick_servo",
        executable="sequence_player",
        name="sequence_player",
        parameters=[{
            "actions": ParameterValue(LaunchConfiguration("actions"), value_type=str),
            "repeat": ParameterValue(LaunchConfiguration("repeat"), value_type=int),
        }],
        output="screen",
        condition=IfCondition(LaunchConfiguration("play")),
    )])

    def _home_node(context, *args, **kwargs):
        """默认不摆 home：上电姿态已经由 driver 的 reset 一次走完。"""
        if not _true(context, "home_enable") or _true(context, "play"):
            return []
        return [TimerAction(period=1.0, actions=[Node(
            package="cquick_servo",
            executable="sequence_player",
            name="home_player",
            parameters=[{
                "actions": ParameterValue(
                    LaunchConfiguration("home"), value_type=str),
                "repeat": 1,
            }],
            output="screen",
        )])]

    return LaunchDescription([
        port_arg,
        demo_arg,
        reset_arg,
        home_arg,
        home_enable_arg,
        play_arg,
        actions_arg,
        repeat_arg,
        driver_node,
        client_node,
        player_node,
        OpaqueFunction(function=_home_node),
    ])
