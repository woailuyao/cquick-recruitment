# =============================================================================
# FieldPose.msg —— 任务2（单目场地自定位）输出
#
# 字段名与考核细则 2.11 节表格**逐字对齐**，评审对照表格看时不会找不到字段。
#
# 坐标系（必须与 field.yaml / field_geometry.yaml / 俯视图一致）：
#   map：+X 向右、+Y 向上（俯视图），单位米；地面为 Z=0
#   yaw：从 +X 轴起逆时针为正，单位度，范围 (-180, 180]
#
# 位姿含义（细则 2.11 明确要求，不能混用）：
#   x/yaw_camera_map  —— 相机**光心**的地图位姿（必填）
#   x/yaw_carrier_map —— 机器人底座/移动测试架参考原点的地图位姿（可选，
#                        由相机外参换算得到；本实现把相机光心直接作为载体
#                        参考原点时，两者数值相同，但仍分别填，避免混淆）
# =============================================================================

std_msgs/Header header

# ---- 相机光心位姿（必填）----
float32 x_camera_map     # m
float32 y_camera_map     # m
float32 yaw_camera_map   # deg，逆时针为正

# ---- 载体参考点位姿（可选）----
float32 x_carrier_map    # m
float32 y_carrier_map    # m
float32 yaw_carrier_map  # deg

# ---- 状态与诊断 ----
bool valid               # 位姿是否可信（status == ok 时为 true）
int32 marker_count       # 参与解算的内点标志数
float32 residual         # 内点残差 RMS，m（-1 表示本帧无有效解）
string status            # ok / insufficient_markers / ambiguous / high_residual / lost
float32 fps              # 实时处理帧率
float32 update_hz        # 位姿更新频率（Hz，位姿有效帧的滑动平均）
float32 processing_ms    # 本帧算法耗时（ms）

# ---- 歧义提示（status == ambiguous 时给出另一个同等合理的解）----
bool has_alternative
float32 alt_x
float32 alt_y
float32 alt_yaw_deg
