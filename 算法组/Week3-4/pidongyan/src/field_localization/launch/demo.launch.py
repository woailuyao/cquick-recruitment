"""可复用 launch：一次拉起足球检测 + 场地定位两个节点。

用法（实验室 Ubuntu + ROS2 Humble）：
  ros2 launch field_localization demo.launch.py
  # 指定权重与场地文件：
  ros2 launch field_localization demo.launch.py \
      ball_model:=/abs/path/best_train_v4.pt \
      marker_model:=/abs/path/markers_best.pt
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="0",
                              description="真实相机索引（验收必须用相机，不能用视频文件）"),
        DeclareLaunchArgument("ball_model", default_value="best_train_v4.pt"),
        DeclareLaunchArgument("marker_model", default_value="",
                              description="四类标志模型；留空则定位节点只发布 valid=false"),
        DeclareLaunchArgument("intrinsics",
                               default_value="config/camera_intrinsics_fieldvideo.yaml"),
        DeclareLaunchArgument("field", default_value="config/field.yaml"),
        DeclareLaunchArgument("h_prior", default_value="0.55"),
        Node(
            package="field_localization",
            executable="ball_detector_node",
            name="ball_detector",
            output="screen",
            parameters=[{
                "source": LaunchConfiguration("source"),
                "model_path": LaunchConfiguration("ball_model"),
                "conf": 0.35,
                "fps": 30.0,
            }],
        ),
        Node(
            package="field_localization",
            executable="field_pose_node",
            name="field_pose",
            output="screen",
            parameters=[{
                "marker_model": LaunchConfiguration("marker_model"),
                "intrinsics": LaunchConfiguration("intrinsics"),
                "field": LaunchConfiguration("field"),
                "h_prior": LaunchConfiguration("h_prior"),
                "pose_every": 5,
            }],
        ),
    ])
