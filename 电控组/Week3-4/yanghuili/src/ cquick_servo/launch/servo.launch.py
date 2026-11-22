"""Launch the current cquick_servo package and safety heartbeat."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    share_dir = get_package_share_directory("cquick_servo")

    default_params = os.path.join(
        share_dir,
        "config",
        "servo_params.yaml",
    )
    default_action = os.path.join(
        share_dir,
        "config",
        "actions",
        "validate_10.yaml",
    )

    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = default_params

    parameters = [params_file]

    port = LaunchConfiguration("port").perform(context)
    if port:
        parameters.append({"port": port})

    driver = Node(
        package="cquick_servo",
        executable="servo_driver",
        name="servo_driver",
        output="screen",
        parameters=parameters,
    )

    heartbeat = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "-r",
            "5",
            "/servo/heartbeat",
            "std_msgs/msg/Bool",
            "{data: true}",
        ],
        emulate_tty=True,
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("heartbeat")
        ),
    )

    action_file = LaunchConfiguration("action_file").perform(context)
    if not action_file:
        action_file = default_action

    player = Node(
        package="cquick_servo",
        executable="sequence_player",
        name="sequence_player",
        output="screen",
        condition=IfCondition(LaunchConfiguration("demo")),
        parameters=[{"action_file": action_file}],
    )

    return [driver, heartbeat, player]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="Optional absolute servo parameter file.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="",
                description="Optional serial port override.",
            ),
            DeclareLaunchArgument(
                "heartbeat",
                default_value="true",
                description="Start the /servo/heartbeat publisher.",
            ),
            DeclareLaunchArgument(
                "demo",
                default_value="false",
                description="Start the YAML sequence player.",
            ),
            DeclareLaunchArgument(
                "action_file",
                default_value="",
                description="YAML action file used when demo:=true.",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
