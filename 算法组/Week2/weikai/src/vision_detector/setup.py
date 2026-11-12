from setuptools import setup

package_name = 'vision_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weikai',
    maintainer_email='weikai@example.com',
    description='目标位置发布/订阅节点（第二周考核）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 编译后可用：ros2 run vision_detector publisher / subscriber
            'publisher = vision_detector.publisher:main',
            'subscriber = vision_detector.subscriber:main',
        ],
    },
)
