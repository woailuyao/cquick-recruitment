# 场地定位结果消息
# 相机光心在场地地图坐标系下的二维位姿（米/度）

std_msgs/Header header

float32 x_camera_map      # 相机光心在地图 X 坐标 (m)
float32 y_camera_map      # 相机光心在地图 Y 坐标 (m)
float32 yaw_camera_map    # 相机水平朝向相对地图 +X 轴 (deg, 逆时针为正)

float32 x_carrier_map     # 载体参考原点地图 X (m, 可选)
float32 y_carrier_map     # 载体参考原点地图 Y (m, 可选)
float32 yaw_carrier_map   # 载体参考原点地图朝向 (deg)

bool valid                # 位姿是否可信
int32 marker_count        # 本帧参与匹配的有效特征数量
float32 residual          # 匹配残差
string status             # ok / insufficient_markers / ambiguous / high_residual / lost
