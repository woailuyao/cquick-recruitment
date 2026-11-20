from setuptools import setup

package_name = "task3_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sumaolin",
    maintainer_email="sumaolin@example.com",
    description="Task3 场地定位的 ROS2 封装（引用 scr/task3.py 与已有权重）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task3_node = task3_ros2.node:main",
            "task3_listener = task3_ros2.listener:main",
        ],
    },
)
