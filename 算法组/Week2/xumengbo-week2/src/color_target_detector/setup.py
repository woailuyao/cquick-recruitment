from setuptools import setup

package_name = 'color_target_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhangsan',
    maintainer_email='you@example.com',
    description='HSV 颜色目标检测 + ROS2 发布/订阅',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 左边是 ros2 run 用的名字，右边是 包名.文件名:函数
            'detect_image = color_target_detector.detect_image:main',
            'param_compare = color_target_detector.param_compare:main',
            'target_publisher = color_target_detector.target_publisher:main',
            'target_subscriber = color_target_detector.target_subscriber:main',
        ],
    },
)
