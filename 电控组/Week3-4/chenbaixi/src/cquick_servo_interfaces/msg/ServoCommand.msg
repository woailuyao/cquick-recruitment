# 一条舵机指令
uint8 id            # 舵机 ID，1~254
uint16 pwm          # 目标 PWM（电气范围 500~2500，实际安全范围由参数决定）
uint16 duration_ms  # 从当前位置走到目标用多久，单位 ms
