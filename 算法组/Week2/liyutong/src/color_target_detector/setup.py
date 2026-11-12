from setuptools import find_packages, setup

package_name = 'color_target_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Color Target Maintainers',
    maintainer_email='maintainer@example.com',
    description='HSV color detection and ROS 2 target position publisher/subscriber.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detect_images = color_target_detector.detect_images:main',
            'target_publisher = color_target_detector.publisher:main',
            'target_subscriber = color_target_detector.subscriber:main',
        ],
    },
)
