from glob import glob
from setuptools import setup

package_name = "color_detector"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 测试图片也装到 share 下，这样装了之后节点还能找到图
        ("share/" + package_name + "/images", glob("data/images/*.jpg")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaojunhao",
    maintainer_email="xjh0911@example.com",
    description="HSV 颜色目标检测与 ROS2 话题发布",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detect_node = color_detector.detect_node:main",
            "listener_node = color_detector.listener_node:main",
            "detect_image = color_detector.detect_image:main",
        ],
    },
)
