# 场地定位结果（对应细则 2.11 的 /localization/field_pose）
# 坐标系：C0 地图；原点=场地中心，+X 指向唯一球门，+Y 为面向 +X 时的左侧，单位米。
# yaw 绕 +Z 逆时针为正，**单位为度**（不是 rad）。
# 载体为独立移动测试架，按细则以相机光心为载体参考原点：
#   x_carrier_map = x_camera_map，输出坐标系 camera_frame。
# valid=false 时必须把 x/y/yaw 留空语义（此处置 0 并置 status），严禁伪造有效位姿。
std_msgs/Header header
float32 x_camera_map
float32 y_camera_map
float32 yaw_camera_map
float32 x_carrier_map
float32 y_carrier_map
float32 yaw_carrier_map
bool valid
int32 marker_count
float32 residual
string status
