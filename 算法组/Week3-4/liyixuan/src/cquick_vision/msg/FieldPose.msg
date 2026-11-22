# cquick_interfaces/msg/FieldPose.msg
# 对应细则 2.11 的字段表；角度单位 deg
std_msgs/Header header

float32 x_camera_map        # 相机光心在地图中的 X，m
float32 y_camera_map        # 相机光心在地图中的 Y，m
float32 yaw_camera_map      # 相机水平朝向相对地图 +X，deg，逆时针为正

float32 x_carrier_map       # 载体参考原点地图 X，m
float32 y_carrier_map       # 载体参考原点地图 Y，m
float32 yaw_carrier_map     # 载体朝向，deg

bool valid
int32 marker_count
float32 residual            # 加权 RMSE，m
string status               # ok / insufficient_markers / ambiguous / high_residual / lost / not_converged
