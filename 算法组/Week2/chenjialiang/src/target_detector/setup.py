from setuptools import setup

package_name = "target_detector"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chenjialiang",
    maintainer_email="14780892013@163.com",
    description="Color-based target detection with ROS2 pub/sub",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_publisher = target_detector.target_publisher:main",
            "target_subscriber = target_detector.target_subscriber:main",
        ],
    },
)
