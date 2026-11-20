# 驱动节点每发完一条指令就往 /servo/state 打一条，作用是：
#   ① bag 回放时能和 /servo/command 对上（命令 → 结果）；
#   ② 安全看门狗靠它的心跳判断"上位机还活着"。

builtin_interfaces/Time stamp   # 发生时间

uint8  id            # 涉及哪个舵机
uint16 pwm_target    # 想让它到哪
uint16 pwm_actual    # PRAD 读回来的实际值；读不到时为 0
bool   ok            # 这条指令算不算成功
string note          # ok=false 时写原因：out_of_range / serial_error / canceled / not_reached:45
