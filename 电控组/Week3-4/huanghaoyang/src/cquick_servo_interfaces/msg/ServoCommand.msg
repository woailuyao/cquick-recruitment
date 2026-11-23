# ServoCommand —— 一条舵机指令的内部表示
#
# 为什么不用 std_msgs：std_msgs 只有值，没有“给谁”。
# 一条能用的指令必须同时说清三件事：给哪个 ID、去哪个 PWM、用多久。
# 这个 msg 只走内部话题（/servo/command 给 bag 回放、/servo/state 给状态显示），
# 真正的控制入口是 Action，不是它。

# 舵机 ID，1~254（本组实际用 1~4）。协议里要补成 3 位，补零在代码里做
uint8 id

# 目标 PWM。这是“电气”允许的 500~2500，不是安全范围
# 安全范围由参数 pwm_min/pwm_max 决定（默认 900~2100），两者别混
uint16 pwm

# 从当前位置走到目标用多久，单位 ms。填 0 表示用 default_duration_ms
uint16 duration_ms
