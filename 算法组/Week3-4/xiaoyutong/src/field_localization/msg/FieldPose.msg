# 单目场地定位的位姿消息
# 对应任务二 2.11 节要求的字段
std_msgs/Header header

# 相机光心在地图中的位姿
float32 x_camera_map      # m
float32 y_camera_map      # m
float32 yaw_camera_map    # deg，从地图 +X 轴逆时针为正

# 载体参考原点在地图中的位姿（本工程与 camera 相同）
float32 x_carrier_map
float32 y_carrier_map
float32 yaw_carrier_map

# 定位状态
bool valid
int32 marker_count
float32 residual          # m，内点平均残差
string status             # ok / insufficient_markers / ambiguous / high_residual / jump_rejected / lost
