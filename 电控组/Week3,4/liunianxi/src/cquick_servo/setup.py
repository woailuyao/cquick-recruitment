from glob import glob
import os

from setuptools import setup

package_name = "cquick_servo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "actions"), glob("config/actions/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="liunianxi",
    maintainer_email="liunianxi@cquick.local",
    description="总线舵机驱动 + 动作组播放",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servo_driver = cquick_servo.servo_driver:main",
            "move_client = cquick_servo.move_client:main",
            "sequence_player = cquick_servo.sequence_player:main",
            "fake_servo = cquick_servo.fake_servo:main",
        ],
    },
)
