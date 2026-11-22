# 内部调试总线 /servo/command 的载荷：与 Action 的 goal 字段一一对应，只为了让 bag 能回放
uint8 id           # 舵机 ID（1~254）
uint16 pwm         # 目标位置（电气范围 500~2500，安全范围由 pwm_min/pwm_max 参数决定）
uint16 duration_ms # 从当前位置走到目标用多久；0 表示用节点的 default_duration_ms
