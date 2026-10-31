from setuptools import find_packages
from setuptools import setup

package_name = 'vision_detector'

setup(
    name=package_name,
    version='0.0.1',
    # 包目录：colcon build 时会把 vision_detector/ 下的 .py 装进 install 空间
    packages=find_packages(exclude=['test']),
    data_files=[
        # 这两行是 ament_python 包的固定写法：
        # 1) 把 resource/<包名> 注册进 ament 索引（ros2 pkg list 能看到本包靠它）
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 2) 把 package.xml 放到 install 空间（依赖解析要用）
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pidongyan',
    maintainer_email='pidongyan@example.com',
    description='第 2 周任务 3：目标位置发布/订阅节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    # console_scripts：注册成 ros2 run 命令。
    # 左边是命令名，右边是 "模块:函数"。
    # 编译 + source 后就能：ros2 run vision_detector target_publisher
    entry_points={
        'console_scripts': [
            'target_publisher = vision_detector.target_publisher:main',
            'target_subscriber = vision_detector.target_subscriber:main',
        ],
    },
)
