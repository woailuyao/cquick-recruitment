# -*- coding: utf-8 -*-
"""ros2 launch week34_vision vision.launch.py

常用参数：
    source:=0                          相机编号（或视频路径）
    size:=640x480                      采集分辨率（WSL 转发相机建议降低）
    fps:=15                            相机帧率，0 表示不设置
    ball_model:=/path/best.pt          足球检测模型（留空则不检测）
    marker_model:=/path/marker.pt      L/T/X 标志检测模型（给了就每帧自动识别）
    marker_map:=X,T,L                  模型类别 id 0,1,2 对应的真实类别
    marker_conf:=0.35                  标志检测置信度阈值
    clicks:=/path/clicks.json          已保存的标志点（没有 marker_model 时使用）
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="0",
                              description="相机编号，或视频文件路径"),
        DeclareLaunchArgument("size", default_value="",
                              description="采集分辨率 WxH，例如 640x480"),
        DeclareLaunchArgument("fps", default_value="0",
                              description="相机帧率，0 表示不设置"),
        DeclareLaunchArgument("ball_model", default_value="",
                              description="足球 YOLO 权重，留空则不检测足球"),
        DeclareLaunchArgument("marker_model", default_value="",
                              description="L/T/X 标志检测模型，留空则人工点选"),
        DeclareLaunchArgument("marker_map", default_value="X,T,L",
                              description="标志模型类别 id 对应的真实类别"),
        DeclareLaunchArgument("marker_conf", default_value="0.35",
                              description="标志检测置信度阈值"),
        DeclareLaunchArgument("clicks", default_value="",
                              description="已保存的标志点 json"),
        Node(
            package="week34_vision",
            executable="vision",
            name="week34_vision",
            output="screen",
            arguments=["--source", LaunchConfiguration("source"),
                       "--size", LaunchConfiguration("size"),
                       "--fps", LaunchConfiguration("fps"),
                       "--ball-model", LaunchConfiguration("ball_model"),
                       "--marker-model", LaunchConfiguration("marker_model"),
                       "--marker-map", LaunchConfiguration("marker_map"),
                       "--marker-conf", LaunchConfiguration("marker_conf"),
                       "--clicks", LaunchConfiguration("clicks"),
                       "--loop"],
        ),
    ])
