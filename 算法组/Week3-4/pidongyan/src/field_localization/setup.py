from setuptools import setup

package_name = "field_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/demo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pidongyan",
    maintainer_email="pidongyan@example.com",
    description="CQuick Week3-4 ball detection + field localization ROS2 nodes",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ball_detector_node = field_localization.ball_detector_node:main",
            "field_pose_node = field_localization.field_pose_node:main",
        ],
    },
)
