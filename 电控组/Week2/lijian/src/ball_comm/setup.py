from setuptools import find_packages, setup

package_name = 'ball_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leyili',
    maintainer_email='leyili@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pub = ball_comm.pub:main',
            'sub = ball_comm.sub:main',
            'nav_turtle = ball_comm.nav_turtle:main',
        ],
    },
)