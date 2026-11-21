"""一键全链路 launch —— 假舵机 + 驱动 + 播放器，一条命令全起来。

    source ~/servo_ws/install/setup.bash
    ros2 launch cquick_servo servo_all.launch.py

三块结构：
  【区域 1】导入 —— 不用动
  【区域 2】★ 配置区 —— 你只改 ★1 和 ★2 两行就够了
  【区域 3】主逻辑 —— 不用动
"""

# =====================================================================
# 【区域 1】导入 —— 不用动
# =====================================================================
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ####################################################################
# #   ★★★【区域 2】配置区 —— 你改的地方在这里 ★★★                  #
# ####################################################################

# ★1 要播哪个动作文件（放 config/actions/ 里）
#    "stand_up.yaml" = 起身下蹲    "wave.yaml" = 波浪
MOTION_FILE = "walk.yaml"

# ★2 真机 or 虚拟：只改这一行！
#    False = 虚拟串口测试（默认，自动起假舵机，串口自动用 /tmp/ttyFAKE）
#    True  = 接真机（不起假舵机，串口自动用 /dev/ttyUSB0）
USE_REAL_SERVO = True

# ↓ 下面这些会自动跟着 ★2 走，一般不用你管 ↓
if USE_REAL_SERVO:
    PORT = "/dev/ttyUSB0"      # 真机串口
else:
    PORT = "/tmp/ttyFAKE"      # 虚拟串口

# ★3 假舵机脚本路径（只有虚拟模式 ★2=False 时才用到）
FAKE_SCRIPT = os.path.expanduser("~/servo_ws/fake_servo.py")

# ★4 波特率、启动延迟、包名、可执行名（都不用动）
BAUD = 115200
START_DELAY = 2.5
PACKAGE_NAME = "cquick_servo"
DRIVER_EXE = "servo_driver"
PLAYER_EXE = "sequence_player"

# ####################################################################
# #                        配置区到此为止                              #
# ####################################################################


# =====================================================================
# 【区域 3】主逻辑 —— 不用动
# 虚拟模式先起假舵机 → 再起驱动 → 再起播放器
# =====================================================================
def generate_launch_description():
    actions = []

    # 第一拨：假舵机（普通 python 脚本，用 ExecuteProcess 跑）
    # 只有虚拟模式（USE_REAL_SERVO=False）才启动它
    if not USE_REAL_SERVO:
        fake = ExecuteProcess(
            cmd=["python3", FAKE_SCRIPT, "--node-side", PORT],
            output="screen",
        )
        actions.append(fake)

    # 第二拨：驱动节点（读 servo_params.yaml，再按 ★2 覆盖串口）
    share = get_package_share_directory(PACKAGE_NAME)
    params_file = os.path.join(share, "config", "servo_params.yaml")
    driver = Node(
        package=PACKAGE_NAME,
        executable=DRIVER_EXE,
        name="servo_driver",
        parameters=[params_file, {"port": PORT, "baudrate": BAUD}],
        output="screen",
    )

    # 第三拨：播放器（把 ★1 的动作文件名传进去）
    player = Node(
        package=PACKAGE_NAME,
        executable=PLAYER_EXE,
        name="sequence_player",
        parameters=[{"motion_file": MOTION_FILE}],
        output="screen",
    )

    # 虚拟模式要给假舵机留时间造串口，真机模式就不用等那么久
    delay = START_DELAY if not USE_REAL_SERVO else 0.5
    actions.append(TimerAction(period=delay, actions=[driver, player]))

    return LaunchDescription(actions)
