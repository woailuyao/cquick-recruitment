# 目标在图像里的位置，供视觉节点发布、订阅节点接收。
# 约定：detected=false 时 u/v/area 一律填 0，接收方必须先看 detected 再取值。

bool detected    # 本帧是否检测到目标
float32 u        # 目标中心的横坐标，单位像素，原点在图像左上角
float32 v        # 目标中心的纵坐标，单位像素，向下为正
float32 area     # 目标轮廓面积，单位像素平方，近似值
