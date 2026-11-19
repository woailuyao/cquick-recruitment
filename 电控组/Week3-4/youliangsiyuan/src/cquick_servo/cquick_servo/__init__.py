# cquick_servo 包的 Python 模块目录
# 这里放三个节点：
#   servo_driver.py   —— Action Server，唯一碰串口的人
#   move_client.py    —— Action Client，键盘手打指令
#   sequence_player.py—— 动作组播放器，读 yaml 展开成 goal 序列
# 外加一个纯函数模块：
#   servo_protocol.py —— 拼指令字符串 / 解析回包（不 import rclpy，pytest 单测对象）
