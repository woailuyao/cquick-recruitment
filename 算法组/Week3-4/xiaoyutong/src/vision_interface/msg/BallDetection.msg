# 单个足球检测结果
std_msgs/Header header
float32 u            # 检测框中心像素 u
float32 v            # 检测框中心像素 v
float32 x1
float32 y1
float32 x2
float32 y2
float32 confidence
string  class_name   # 固定为 "ball"
