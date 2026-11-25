import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("cquick_servo")
    default_params = os.path.join(pkg_share, "config", "servo_params.yaml")

    params_file = LaunchConfiguration("params_file")
    port = LaunchConfiguration("port")
    demo = LaunchConfiguration("demo")
    watchdog = LaunchConfiguration("watchdog")

    driver = Node(
        package="cquick_servo",
        executable="servo_driver",
        name="servo_driver",
        parameters=[
            params_file,
            {"port": ParameterValue(port, value_type=str)},
            # 加分项开关：默认关，watchdog:=true 时启动即开启
            {"watchdog_enabled": ParameterValue(watchdog, value_type=bool)},
        ],
        output="screen",
    )

    client = Node(
        package="cquick_servo",
        executable="move_client",
        name="move_client",
        arguments=["1", "1700", "1000"],
        condition=IfCondition(demo),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("port", default_value="/dev/cq_servo"),
        DeclareLaunchArgument("demo", default_value="false"),
        DeclareLaunchArgument(
            "watchdog", default_value="false",
            description="启动时是否打开安全看门狗（演示期间保持 false）"),
        driver,
        client,
    ])
