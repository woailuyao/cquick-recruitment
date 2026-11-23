uint8 id             # 舵机 ID，0~254（本机实测为 000）
uint16 pwm           # 目标 PWM，电气极限 500~2500，不是安全范围
uint16 duration_ms   # 从当前位置走到目标用多久，ms
