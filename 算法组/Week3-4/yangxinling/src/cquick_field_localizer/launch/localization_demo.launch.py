#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localization_demo.launch.py —— 任务2 现场演示的一键启动

三种画面来源，用参数切换：

    1) 真机相机（默认）：
           ros2 launch cquick_field_localizer localization_demo.launch.py

    2) 用录好的视频当输入（无相机也能演示 / 可复现测试）：
           ros2 launch cquick_field_localizer localization_demo.launch.py \
               video:="video/场地/B点.avi"

    3) 明确不启动任何图像源（图像由别的节点提供）：
           ros2 launch cquick_field_localizer localization_demo.launch.py source:=none

参数：
    config      配置文件（默认 config/camera.yaml，可用绝对路径）
    image_topic 图像话题
    source      camera | video | none
    video       视频路径（source=video 时必填）
    show        是否弹窗（默认 true，现场演示要开）
    out_video   把分屏画面录成 mp4（默认空 = 不录）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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
    use_ci = LaunchConfiguration("use_camera_info").perform(context)
    prior = LaunchConfiguration("prior").perform(context)

    actions = []
    if source == "camera":
        # 直接用自己的 video_publisher 开 /dev/videoN：
        #   - 不依赖 usb_cam（这台机器没装，apt 也不一定连得上）；
        #   - 相机节点和定位节点用同一个 OpenCV，取到的分辨率/像素格式完全一致，
        #     不会再出现"usb_cam 发 yuyv、下游按 bgr8 解释导致变色"的坑。
        # 如果你的相机需要特定像素格式/曝光控制，把 camera_device 换成自己的驱动节点即可。
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
        package="cquick_field_localizer", executable="field_localizer",
        additional_env=_py_env(),
        name="field_localizer", output="screen",
        parameters=[{
            "config": cfg,
            "image_topic": topic,
            "show": show.lower() in ("1", "true", "yes"),
            "out_video": out_video,
            "use_camera_info": use_ci.lower() in ("1", "true", "yes"),
            "use_prior": bool(prior.strip()),
            "prior_x": float(prior.split(",")[0]) if prior.strip() else 0.0,
            "prior_y": float(prior.split(",")[1]) if prior.strip() else 0.0,
            "prior_yaw_deg": float(prior.split(",")[2]) if prior.strip() else 0.0,
        }],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="config/camera.yaml",
                              description="配置文件路径"),
        DeclareLaunchArgument("image_topic", default_value="/image_raw"),
        DeclareLaunchArgument("source", default_value="camera",
                              description="camera | video | none"),
        DeclareLaunchArgument("video", default_value=""),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0",
                              description="source:=camera 时打开的相机设备"),
        DeclareLaunchArgument("show", default_value="true"),
        DeclareLaunchArgument("out_video", default_value=""),
        DeclareLaunchArgument("use_camera_info", default_value="true"),
        DeclareLaunchArgument("prior", default_value="",
                              description="初始位姿先验 x,y,yaw_deg"),
        OpaqueFunction(function=_launch_setup),
    ])
