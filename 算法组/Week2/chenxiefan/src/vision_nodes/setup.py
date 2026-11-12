from setuptools import setup

package_name = 'vision_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chengdousheshou',
    maintainer_email='348557060+chengdousheshou@users.noreply.github.com',
    description='颜色目标检测的 ROS 2 发布与订阅节点',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = vision_nodes.publisher_node:main',
            'subscriber_node = vision_nodes.subscriber_node:main',
        ],
    },
)
