# 目标位置消息（第 2 周任务 3，字段与考核细则 3.1 完全一致）
#
# .msg 文件格式：每行 "类型 字段名"，# 后面是注释。
# colcon 编译时 rosidl 会把它自动生成 Python/C++ 类
# （Python 里就是 from vision_interfaces.msg import TargetPosition）。

# 本帧是否检测到目标。
# 这是整条消息的"有效开关"：接收方必须先判断它！
# detected=false 时 u/v/area 都是无意义的默认值 0，
# 绝不能把 (0,0) 理解成"目标在图像左上角"。
bool detected

# 目标中心的图像横坐标（像素，原点在左上角，向右增大）
float32 u

# 目标中心的图像纵坐标（像素，原点在左上角，向下增大）
float32 v

# 目标轮廓面积（像素平方，近似值）
# 注意单位是像素^2 不是平方米，要换算真实大小需要相机标定，本周不要求
float32 area
