from setuptools import find_packages, setup

package_name = "cquick_color_vision"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/two_nodes.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yangtiancheng",
    maintainer_email="student@example.com",
    description="HSV color detection and ROS 2 target-position communication.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "detect=cquick_color_vision.detector:main",
            "visualize=cquick_color_vision.visualize:main",
            "compare_params=cquick_color_vision.parameter_compare:main",
            "make_test_images=cquick_color_vision.make_test_images:main",
            "publish_target=cquick_color_vision.publisher_node:main",
            "subscribe_target=cquick_color_vision.subscriber_node:main",
        ],
    },
)
