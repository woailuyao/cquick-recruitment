from glob import glob
import os

from setuptools import setup

package_name = "cquick_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        # ament 通过这个文件认领包
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="liuzikang",
    maintainer_email="liuzikang@example.com",
    description="蓝色目标检测 + /target_position 发布订阅",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_publisher = cquick_vision.target_publisher:main",
            "target_subscriber = cquick_vision.target_subscriber:main",
            "camera_publisher = cquick_vision.camera_publisher:main",
        ],
    },
)
