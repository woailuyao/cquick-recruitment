# 驱动节点每发完一条指令就往 /servo/state 打一条，作用是：
#   ① bag 回放时能和 /servo/command 对上（命令 → 结果）；
#   ② 安全看门狗靠它的心跳判断"上位机还活着"。

# 时间戳放在最前面：bag 回放与日志对齐都靠它。
builtin_interfaces/Time stamp   # 发生时间

# 与 ServoCommand.id 对应，方便按舵机筛选。
uint8  id            # 涉及哪个舵机
uint16 pwm_target    # 想让它到哪
# 实际值只有读回成功才有意义；0 是“没读到”的约定值。
uint16 pwm_actual    # PRAD 读回来的实际值；读不到时为 0
# ok 与 note 配合：ok=true 时 note 通常是 ok(diff=…)，ok=false 时 note 是失败原因。
bool   ok            # 这条指令算不算成功
string note          # ok=false 时写原因：out_of_range / serial_error / canceled / not_reached:45
