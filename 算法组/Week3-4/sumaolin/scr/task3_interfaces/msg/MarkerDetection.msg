# 一个场地标志的检测结果
#
# type 取值：L / T / X
string type

# 检测框中心的像素坐标
float32 u
float32 v
float32 confidence

# plane 模式下有载体坐标系地面坐标；pnp 模式没有，has_ground_xy=False
bool has_ground_xy
float32 x
float32 y
