# 一条舵机指令（同时用于 /servo/command 话题，给 bag 回放用）
# 注意：pwm 这里是电气极限范围 500~2500，安全范围由驱动节点的 pwm_min/pwm_max 参数决定
uint8 id
uint16 pwm
uint16 duration_ms
