# 目标位置消息：发布节点 -> /target_position
# 约定：detected=false 时 u/v/area 一律为 0（默认值），
#       订阅方必须靠 detected 判断有效性，不能把 0 当成「目标在左上角」。

bool detected      # 是否检测到目标（有效性标志，最重要）
float32 u          # 中心点水平像素坐标（向右为正，左上角原点）
float32 v          # 中心点垂直像素坐标（向下为正，左上角原点）
float32 area       # 目标轮廓面积（像素）
