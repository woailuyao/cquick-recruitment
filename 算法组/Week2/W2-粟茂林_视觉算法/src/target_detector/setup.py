from glob import glob
import os
from setuptools import setup

package_name = "target_detector"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        # 这三行是 ament_python 包的固定套路，缺一行 ros2 run 就找不到包
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 把测试图片装进 share 目录，节点就不用依赖 /mnt/c 上的中文路径
        (os.path.join("share", package_name, "data"), glob("data/*.jpg")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sumaolin",
    maintainer_email="sumaolin@example.com",
    description="HSV 白色足球检测并发布到 /target_position",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        # 左边是 ros2 run 用的可执行名，右边是 "包名.模块:函数"
        "console_scripts": [
            "target_publisher = target_detector.target_publisher:main",
            "target_subscriber = target_detector.target_subscriber:main",
        ],
    },
)
