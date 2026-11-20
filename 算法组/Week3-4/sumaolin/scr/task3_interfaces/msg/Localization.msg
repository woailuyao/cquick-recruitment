# Task3 单帧定位结果
std_msgs/Header header

# 位姿是否有效（PnP 至少 4 个标志、plane 至少 3 个标志）
bool valid

# 是否由上一帧位姿做时序消歧选出
bool temporal

# ok / insufficient_markers / ambiguous / high_residual / not_converged
string status

# pnp 或 plane
string pose_mode

# 实际参与位姿求解的内点数
uint8 marker_count

# 位姿残差；pnp 单位 px，plane 单位 m
float32 residual
string residual_unit

# 相机光心在地图坐标系中的位置与偏航角（deg）
float32 x_camera_map
float32 y_camera_map
float32 yaw_camera_map

float32 fps

MarkerDetection[] markers
BallDetection[] balls
