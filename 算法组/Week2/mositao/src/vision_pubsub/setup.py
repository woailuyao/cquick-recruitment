from setuptools import setup

package_name = 'vision_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mositao',
    maintainer_email='mositao@example.com',
    description='Publisher/subscriber of TargetPosition (Week 2)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector_node = vision_pubsub.detector_node:main',
            'subscriber_node = vision_pubsub.subscriber_node:main',
        ],
    },
)
