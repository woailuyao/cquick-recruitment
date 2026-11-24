from glob import glob
import os

from setuptools import setup

package_name = "cquick_servo"

setup(
    name=package_name,
    version="0.1.0",
    # tools 子包必须显式列出，否则安装时不带进 site-packages，ros2 run 会找不到。
    packages=[package_name, f"{package_name}.tools"],
    # data_files 决定“装完之后 share/ 里有什么”：launch、config、actions 全靠它，否则 ros2 launch / 参数文件都找不到。
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # 动作组 yaml 单独一条：要保留 actions/ 这个子目录结构，否则 yaml 会平铺在一起。
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
            # entry_points 是 `ros2 run cquick_servo servo_driver` 能用的前提。
            "servo_driver = cquick_servo.servo_driver:main",
            "move_client = cquick_servo.move_client:main",
            "sequence_player = cquick_servo.sequence_player:main",
            "fake_servo = cquick_servo.fake_servo:main",
            "cancel_demo = cquick_servo.tools.cancel_demo:main",
            # 工具脚本也注册成命令：现场直接 ros2 run cquick_servo scan_ids，不用记文件路径。
            "scan_ids = cquick_servo.tools.scan_ids:main",
        ],
    },
)
