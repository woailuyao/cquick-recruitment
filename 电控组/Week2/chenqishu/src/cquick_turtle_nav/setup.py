from setuptools import setup

package_name = 'cquick_turtle_nav'

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
    maintainer='chenqishu',
    maintainer_email='chenqishu@todo.todo',
    description='第2周任务2：订阅 ball_pos，按 1->2->3 闭环走到三个球坐标',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_chaser = cquick_turtle_nav.ball_chaser:main',
        ],
    },
)
