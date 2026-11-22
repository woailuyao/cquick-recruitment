# 单目场地自定位结果（任务 2，固定外参链路）
# 发布话题：/localization/field_pose
# 字段命名与细则 2.11 建议表一致（x/y_carrier_map 为可选项，未实现——
# 本任务载体与相机刚性连体，相机光心即采样参考点，无需二次换算）
# 坐标约定：地图坐标系 +X 指向球门、+Y 为面向 +X 时的左侧；
#          单位米；yaw 从 +X 轴起逆时针为正（单位：度）
# 注意：valid=false 时位置字段是最后一次有效估计，必须先看 valid

std_msgs/Header header
bool valid                # 定位是否有效（残差达标且解算成功）
float32 x_camera_map      # 相机光心在地图中的 X（米；固定外参链路直接解出，即光心地面投影）
float32 y_camera_map      # 相机光心在地图中的 Y（米）
float32 yaw_camera_map    # 相机朝向相对地图 X 轴的角度（度，+X 起逆时针为正）
uint32 marker_count       # 本帧检测到的 L/T 标志数
float32 residual          # 当前解的米制残差（米）
string status             # OK / INVALID: no landmark detected / SEARCHING: high residual 等
uint32 frame_seq          # 处理帧序号（自增）
float32 fps               # 当前处理帧率
