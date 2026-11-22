# 一条舵机指令：内部调试总线 /servo/command 使用，也是 ros2 bag 回放的数据来源
# 注意：本机舵机实测 ID 从 0 开始（出厂 ID=0），uint8 合法范围 0~255

uint8 id            # 舵机 ID
uint16 pwm          # 目标 PWM。电气极限 500~2500，安全范围由参数 pwm_min/pwm_max 决定
uint16 duration_ms  # 从当前位置走到目标用时，ms；0 表示使用参数 default_duration_ms
