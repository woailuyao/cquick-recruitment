# 舵机指令（内部调试总线用，bag 录制/回放）
uint8 id          # 舵机 ID，1~254
uint16 pwm        # 目标 PWM（电气范围 500~2500）
uint16 duration_ms  # 从当前位置走到目标用多久
