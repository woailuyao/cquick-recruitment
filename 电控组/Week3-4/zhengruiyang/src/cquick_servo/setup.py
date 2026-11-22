from setuptools import find_packages, setup
from glob import glob      # glob 用来按通配符（*）找文件
package_name = 'cquick_servo'

setup(
    name=package_name,          # 包名
    version='0.0.0',            # 版本号（自己写，随便填）
    # 自动找到内层的 cquick_servo/ 目录（里面有 __init__.py）当成 Python 包
    packages=find_packages(exclude=['test']),
    # data_files = 把"非 Python 文件"（launch、config、yaml）手动拷进 install 的 share/ 目录
    # 不写这些，launch 和 yaml 不会进 install，ros2 launch 就找不到文件（Q4 高频坑）
    data_files=[
        # 包标记文件（ROS2 靠这个文件认出"这是个已安装的包"）
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml 本身也要装进去
        ('share/' + package_name, ['package.xml']),
        # launch/ 下所有 .launch.py 装到 share/<包名>/launch/
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # config/ 下所有 .yaml 装到 share/<包名>/config/
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # config/actions/ 下的动作 yaml 装到 share/<包名>/config/actions/
        ('share/' + package_name + '/config/actions', glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],   # 打包工具依赖
    zip_safe=True,
    maintainer='zhengruiyang',
    maintainer_email='fei_maozai@qq.com',
    description='CQuick servo driver and action server',
    license='Apache-2.0',
    tests_require=['pytest'],
    # entry_points = 把 .py 里的 main 函数注册成"可执行命令"
    # 左边 'servo_driver' 是命令名（对应 ros2 run cquick_servo servo_driver），
    # 右边 'cquick_servo.servo_driver:main' 是"模块路径:函数名"
    entry_points={
        'console_scripts': [
            'servo_driver = cquick_servo.servo_driver:main',
            'move_client = cquick_servo.move_client:main',
            'sequence_player = cquick_servo.sequence_player:main',
            'watchdog = cquick_servo.watchdog:main',
        ],
    },
)
