# 一条总线舵机指令（内部调试总线 /servo/command 也用这个类型）
uint8 id            # 舵机 ID，1~254
uint16 pwm          # 目标 PWM，电气范围 500~2500，安全范围由参数 pwm_min/pwm_max 决定
uint16 duration_ms  # 从当前位置走到目标用多久，单位 ms
