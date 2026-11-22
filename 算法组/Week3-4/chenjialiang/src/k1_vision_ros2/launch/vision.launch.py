# 启动定位与足球检测两个节点，可按参数单独关闭
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    url = LaunchConfiguration("camera_url")
    run_ball = LaunchConfiguration("run_ball")
    run_pose = LaunchConfiguration("run_pose")
    ball_model = LaunchConfiguration("ball_model")
    pose_model = LaunchConfiguration("pose_model")

    return LaunchDescription([
        DeclareLaunchArgument("camera_url", default_value="http://localhost:8000/stream"),
        DeclareLaunchArgument("run_ball", default_value="true"),
        DeclareLaunchArgument("run_pose", default_value="true"),
        DeclareLaunchArgument("ball_model", default_value=""),
        DeclareLaunchArgument("pose_model", default_value=""),
        Node(
            package="k1_vision_ros2",
            executable="ball_node",
            name="k1_vision_ball",
            output="screen",
            condition=IfCondition(run_ball),
            parameters=[{"camera_url": url, "model": ball_model}],
        ),
        Node(
            package="k1_vision_ros2",
            executable="pose_node",
            name="k1_vision_pose",
            output="screen",
            condition=IfCondition(run_pose),
            parameters=[{"camera_url": url, "model": pose_model}],
        ),
    ])
