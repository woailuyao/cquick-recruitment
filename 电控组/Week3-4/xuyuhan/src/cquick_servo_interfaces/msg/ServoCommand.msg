# 一条舵机指令（内部调试总线用，也是 bag 回放的载体）
uint8 id             # 舵机 ID，1~254
uint16 pwm           # 目标 PWM。注意：这是电气极限范围，不是结构安全范围
uint16 duration_ms   # 从当前位置走到目标用多久，单位 ms
