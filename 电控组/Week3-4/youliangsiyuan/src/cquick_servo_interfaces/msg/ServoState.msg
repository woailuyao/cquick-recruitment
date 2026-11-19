# 驱动节点对外广播的状态 / 心跳 —— 内部调试总线 /servo/state 用的消息类型
# 两个作用：
#   1. 让 ros2 bag 除了"发了什么"还能录到"当时串口在不在、在跑什么阶段"；
#   2. 给外部监控/看门狗一个可以订阅的心跳（1 秒没有新消息 = 驱动节点可能卡死了）。
#
# ⚠️ 这个心跳只能证明"驱动节点还活着"，不能代替节点内部的堵转看门狗：
#    卸力必须由持有串口的进程自己去做，外面的人没法替它发 PULK。

builtin_interfaces/Time stamp  # 发布这一刻的时间戳

bool   serial_online  # 串口当前是否可用（拔了 USB / 打开失败 会变 false）
string port           # 当前实际在用的串口名（从 port 参数读出来，方便核对）
uint8  last_id        # 最近一次下发的舵机 ID（没有就 0）
uint16 last_pwm       # 最近一次下发的 PWM（没有就 0）
string phase          # idle / torque_on / moving / canceled / error
