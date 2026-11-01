# 这里写 setup()：packages、data_files（ament_index + package.xml）、
# entry_points 的 console_scripts（detector / listener 两个命令名）
from setuptools import setup  # 标准Python打包

package_name = "target_detector"

setup(
    name = package_name,#发行包名
    version = "0.0.0",#版本号
    packages = [package_name],#python目录
    data_files = [
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),  #路径
        ("share/" + package_name, ["package.xml"]),                                   # 把 package.xml 装进去
    ],
    install_requires=["setuptools"],#安装依赖
    zip_safe = True,
    maintainer = "huangyirui",
    maintainer_email = "huangyirui@example.com",
    description = "第 2 周考核之目标位置发布与订阅",
    license="Apache-2.0",#声明授权
    tests_require = ["pytest"],
    entry_points = {
        "console_scripts": [                       #给ros2run一个路径
            "detector = target_detector.detector_node:main",    # ros2 run target_detector detector
            "listener = target_detector.listener_node:main",    # ros2 run target_detector listener
        ],
    },
)