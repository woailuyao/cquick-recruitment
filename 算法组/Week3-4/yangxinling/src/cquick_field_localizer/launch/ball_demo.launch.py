#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ball_demo.launch.py —— 任务1 现场演示的一键启动

    ros2 launch cquick_field_localizer ball_demo.launch.py
    ros2 launch cquick_field_localizer ball_demo.launch.py video:="video/足球/滚动.avi"

参数与 localization_demo.launch.py 一致（source: camera | video | none）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _py_env():
    """让节点用 venv 里的 onnxruntime/ultralytics/numpy。

    colcon 生成的 console script 的 shebang 是 /usr/bin/python3（系统解释器），
    所以要在 PYTHONPATH 前面插 venv 的 site-packages，否则会报
    'No module named onnxruntime' 然后退化成无检测模式。
    用法：
        export CQUICK_PYTHONPATH=$(~/cquick_venv/bin/python -c
            'import site;print(site.getsitepackages()[0])')
    """
    extra = os.environ.get("CQUICK_PYTHONPATH", "").strip()
    cur = os.environ.get("PYTHONPATH", "").strip()
    if extra:
        return {"PYTHONPATH": extra + ((":" + cur) if cur else "")}
    return {}


def _launch_setup(context, *args, **kwargs):
    cfg = LaunchConfiguration("config").perform(context)
    topic = LaunchConfiguration("image_topic").perform(context)
    source = LaunchConfiguration("source").perform(context)
    video = LaunchConfiguration("video").perform(context)
    show = LaunchConfiguration("show").perform(context)
    out_video = LaunchConfiguration("out_video").perform(context)

    actions = []
    if source == "camera":
        # 与 localization_demo.launch.py 同样的理由：用自带 video_publisher 开相机，
        # 不依赖 usb_cam，像素格式和下游解码严格一致。
        dev = LaunchConfiguration("camera_device").perform(context).strip()
        dev_idx = dev.replace("/dev/video", "").strip() if dev.startswith("/dev/video") else dev
        try:
            dev_idx = int(dev_idx)
        except ValueError:
            dev_idx = 0
        actions.append(Node(
            package="cquick_field_localizer", executable="video_publisher",
            additional_env=_py_env(),
            name="camera_publisher", output="screen",
            parameters=[{
                "device": dev_idx,
                "width": 1280, "height": 720, "fps": 30.0,
                "topic": topic,
            }],
        ))
    elif source == "video":
        if not video:
            raise RuntimeError("source:=video 时必须给 video:=<路径>")
        actions.append(Node(
            package="cquick_field_localizer", executable="video_publisher",
            additional_env=_py_env(),
            output="screen",
            parameters=[{"video": video, "topic": topic, "loop": True}],
        ))

    actions.append(Node(
        package="cquick_field_localizer", executable="ball_node",
        additional_env=_py_env(),
        name="ball_node", output="screen",
        parameters=[{
            "config": cfg,
            "image_topic": topic,
            "show": show.lower() in ("1", "true", "yes"),
            "out_video": out_video,
        }],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="config/camera.yaml"),
        DeclareLaunchArgument("image_topic", default_value="/image_raw"),
        DeclareLaunchArgument("source", default_value="camera",
                              description="camera | video | none"),
        DeclareLaunchArgument("video", default_value=""),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0",
                              description="source:=camera 时打开的相机设备"),
        DeclareLaunchArgument("show", default_value="true"),
        DeclareLaunchArgument("out_video", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
