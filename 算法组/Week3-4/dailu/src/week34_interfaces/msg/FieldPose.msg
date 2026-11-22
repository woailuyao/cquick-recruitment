# 场地定位结果（任务 2）
std_msgs/Header header

# 相机光心在地图坐标系中的位姿
float32 x_camera_map       # m
float32 y_camera_map       # m
float32 yaw_camera_map     # deg，相对地图 +X 轴逆时针为正

# 载体参考原点（本次以相机光心正下方地面点为载体原点，故与上面相同）
float32 x_carrier_map      # m
float32 y_carrier_map      # m
float32 yaw_carrier_map    # deg

# 状态
bool    valid              # 位姿是否可信
int32   marker_count       # 本帧有效参与匹配的标志数
float32 residual           # 匹配残差 RMS，m
string  status             # ok / insufficient_markers / ambiguous / high_residual / lost
