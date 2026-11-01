from glob import glob
import os

from setuptools import setup

package_name = "green_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 注意不能写 "launch/*.launch.py"：那样只匹配 xxx.launch.py，
        # 本包的文件叫 detect_launch.py，会匹配不到 -> 编译不报错但 launch 目录是空的，
        # ros2 launch 才报 file not found。用 "launch/*.py" 最稳。
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "test_images"),
         glob("test_images/*.jpg") + glob("test_images/*.jpeg") + glob("test_images/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="houkehong",
    maintainer_email="houkehong@example.com",
    description="第2周：HSV 蓝色目标检测 + ROS2 目标位置发布",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detect_image = green_detector.detect_image:main",
            "batch_test = green_detector.batch_test:main",
            "compare_params = green_detector.compare_params:main",
            "publish_target = green_detector.publish_target:main",
            "subscribe_target = green_detector.subscribe_target:main",
            "fake_publisher = green_detector.fake_publisher:main",
            "check_env = green_detector.check_env:main",
        ],
    },
)
