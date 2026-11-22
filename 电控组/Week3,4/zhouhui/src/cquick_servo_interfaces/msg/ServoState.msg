# 心跳：驱动节点每 0.2 s 发一次，bag 里能看出节点活着的证据
bool alive          # 串口链路是否可用
bool locked         # 安全锁（看门狗触发后为 true，必须人工复位）
uint8 online_count  # 当前认为在线的舵机数量
string detail       # 一句话状态，例如 "link=ok goals=2"
