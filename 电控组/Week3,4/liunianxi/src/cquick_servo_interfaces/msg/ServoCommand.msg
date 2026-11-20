# 一条完整的舵机指令。
# 这是"内部调试总线"的载荷：Action 内部要用它，/servo/command 话题也用它，
# 好处是 bag 回放时一条消息就能还原一次动作。

uint8 id            # 舵机 ID，1~254
uint16 pwm          # 目标 PWM。注意：这是电气极限范围（500~2500），不是安全范围；
                    # 安全范围由驱动节点的参数 pwm_min / pwm_max 决定。
uint16 duration_ms  # 从当前位置走到目标用多久，单位 ms。给太小会猛跳撞结构。
