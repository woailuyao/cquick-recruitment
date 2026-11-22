from glob import glob
import os

from setuptools import setup

package_name = "cquick_field_localizer"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (# launch 文件装进 share，方便 ros2 launch
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Week3-4 招新考核：足球检测 + 单目场地自定位",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 离线可视化（不依赖 ROS2，先跑通这个最容易看到效果）
            "localize = cquick_field_localizer.localize:main",
            # ROS2 节点
            "ball_node = cquick_field_localizer.ball_node:main",
            "field_localizer = cquick_field_localizer.localizer_node:main",
            "video_publisher = cquick_field_localizer.video_publisher:main",
        ],
    },
)
