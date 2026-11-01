# 一帧图像中颜色目标的位置信息
# detected=false 时 u、v、area 约定为 0.0

bool detected     # 本帧是否检测到目标
float32 u         # 目标中心横坐标（像素，向右增大）
float32 v         # 目标中心纵坐标（像素，向下增大）
float32 area      # 目标轮廓面积（像素平方，近似值）
