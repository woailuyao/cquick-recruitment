# /servo/state 心跳：每个舵机一条，bag 回看和看门狗都靠它
uint8 id
uint16 pwm            # 最近一次 PRAD 读回的位置（读不到就填上次下发的目标）
bool torque_enabled   # 是否已发过 PULR
bool online           # 最近一次查询有没有回显
