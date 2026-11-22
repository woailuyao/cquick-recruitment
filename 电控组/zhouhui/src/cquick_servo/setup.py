from glob import glob
import os

from setuptools import setup

package_name = "cquick_servo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # launch/ 和 config/ 默认不会进 install，必须显式写；改完要重新 colcon build
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "actions"),
         glob("config/actions/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="you@example.com",
    description="CQuick bus-servo driver, action client and yaml-driven sequence player",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servo_driver = cquick_servo.servo_driver:main",
            "move_client = cquick_servo.move_client:main",
            "sequence_player = cquick_servo.sequence_player:main",
        ],
    },
)
