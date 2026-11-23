from setuptools import find_packages, setup

package_name = 'target_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/images',
            ['images/test4.png', 'images/test5.png', 'images/test6.png',
             'images/test7.png', 'images/test8.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keventhen',
    maintainer_email='keventhen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_pub = target_publisher.target_pub:main',
        ],
    },
)
