from glob import glob
import os

from setuptools import setup

package_name = "week34_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dailu",
    maintainer_email="dailu@example.com",
    description="CQuick week3-4 vision node",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vision = week34_vision.main:main",
        ],
    },
)
