from glob import glob
import os

from setuptools import setup

package_name = "k1_vision_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chenjialiang",
    maintainer_email="chenjialiang@example.com",
    description="K1 单目视觉：足球检测与场地自定位",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pose_node = k1_vision_ros2.pose_node:main",
        ],
    },
)
