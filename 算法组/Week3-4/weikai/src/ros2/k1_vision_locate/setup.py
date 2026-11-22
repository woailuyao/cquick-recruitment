from setuptools import find_packages, setup

package_name = "k1_vision_locate"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/demo_launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="weikai",
    maintainer_email="weikai@example.com",
    description="CQuick 第3-4周视觉定位 ROS2 节点（足球检测 + 单目场地自定位）",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vision_node = k1_vision_locate.vision_node:main",
        ],
    },
)
