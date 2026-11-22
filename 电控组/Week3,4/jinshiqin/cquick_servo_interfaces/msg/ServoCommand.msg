# 内部调试总线 /servo/command 用，同时被 MoveServo.action 的 Goal 嵌套
uint8 id            # 舵机 ID，1~254
uint16 pwm          # 目标 PWM（500~2500 电气范围；安全范围由参数判定）
uint16 duration_ms  # 走到目标用多久，0 表示用驱动的默认值
