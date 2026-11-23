# 目标位置消息
# 生成的头文件/模块：cquick_interfaces/msg/target_position.hpp (C++) 或
#                     cquick_interfaces.msg.TargetPosition (Python)
#
# 字段顺序、类型必须和考核细则一致，评审会用
#   ros2 interface show cquick_interfaces/msg/TargetPosition
# 来核对这 4 个字段。

bool detected    # 本帧是否检测到目标。False 时下面的数值全部无效
float32 u        # 目标中心的图像横坐标（像素），图像左上角为 0，向右增大
float32 v        # 目标中心的图像纵坐标（像素），图像左上角为 0，向下增大
float32 area     # 目标轮廓面积（像素平方，近似值）
