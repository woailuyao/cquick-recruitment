# 一条完整的舵机位置指令
# 用途：内部调试总线 /servo/command 的消息体，也是 ros2 bag 回放的载体
#       与 MoveServo.action 的 Goal 字段保持一致，便于命令行互相对照

uint8 id           # 舵机 ID，范围 1~254；必须出现在节点参数 servo_ids 里
uint16 pwm         # 目标 PWM，电气极限 500~2500，实际安全区间由参数 pwm_min/pwm_max 约束
uint16 duration_ms # 运动时间(ms)，0 表示使用节点参数 default_duration_ms
