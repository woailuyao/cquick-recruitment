from setuptools import setup

package_name = 'mine_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xueziyi',
    maintainer_email='xueziyi@example.com',
    description='视觉目标检测的 ROS2 发布/订阅节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run mine_detector publisher  → 调用 publisher_node.py 的 main()
            'publisher = mine_detector.publish:main',
            'subscriber = mine_detector.subscriber:main',
        ],
    },
)
