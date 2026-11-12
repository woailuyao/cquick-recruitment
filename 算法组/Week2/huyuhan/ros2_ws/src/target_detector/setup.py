from setuptools import find_packages, setup

package_name = "target_detector"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hu Yuhan",
    maintainer_email="hu_yuhan@todo.todo",
    description="Color target detection and ROS2 communication",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "target_publisher = target_detector.target_publisher:main",
            "target_subscriber = target_detector.target_subscriber:main",
        ],
    },
)
