"""一键启动：驱动节点 + yaml 参数，客户端按需启动。

  ros2 launch cquick_servo servo.launch.py                    # 只起驱动
  ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1 # 换端口（覆盖 yaml）
  ros2 launch cquick_servo servo.launch.py demo:=true         # 顺带起客户端
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成节点描述表。"""
    port_arg = DeclareLaunchArgument("port", default_value="/dev/cq_servo")
    demo_arg = DeclareLaunchArgument("demo", default_value="false")

    pkg_share = FindPackageShare(package="cquick_servo").find("cquick_servo")
    yaml_path = PathJoinSubstitution([pkg_share, "config", "servo_params.yaml"])

    driver_node = Node(
        package="cquick_servo",
        executable="servo_driver",
        name="servo_driver",
        parameters=[yaml_path, {
            # value_type=str 不能省，否则 /dev/ttyUSB1 会被当数字解析
            "port": ParameterValue(LaunchConfiguration("port"), value_type=str),
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

    return LaunchDescription([
        port_arg,
        demo_arg,
        driver_node,
        client_node,
    ])
