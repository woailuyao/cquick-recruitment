from glob import glob

from setuptools import setup

package_name = 'target_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 测试图片一起装进 share 目录，节点用 ament_index 找得到，
        # 这样不管在哪个工作目录下启动都不会找不到图
        ('share/' + package_name + '/images', glob('images/*.jpg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='szc',
    maintainer_email='szc@example.com',
    description='HSV 颜色目标检测 + ROS2 发布/订阅',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = target_vision.detector_node:main',
            'listener_node = target_vision.listener_node:main',
        ],
    },
)
