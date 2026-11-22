from setuptools import setup
import os
from glob import glob

package_name = "k1_localization_ros"

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
    maintainer="HouKehong",
    maintainer_email="houkehong@example.com",
    description="K1 monocular field localization ROS2 interface",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = k1_localization_ros.bridge_node:main",
            "pose_publisher = k1_localization_ros.pose_publisher:main",
        ],
    },
)
