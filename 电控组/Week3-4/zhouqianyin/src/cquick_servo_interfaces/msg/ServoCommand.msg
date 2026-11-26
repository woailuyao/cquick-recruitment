uint8 id           # 舵机 ID，1~254
uint16 pwm         # 目标 PWM，安全区间由参数 pwm_min/pwm_max 约束
uint16 duration_ms # 运动时间(ms)，0 = 用参数默认值
