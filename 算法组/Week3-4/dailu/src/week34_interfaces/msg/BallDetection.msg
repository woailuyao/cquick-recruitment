# 足球检测结果（任务 1）
std_msgs/Header header

bool detected          # 本帧是否检测到球
int32 count            # 检测到的球数（一般 0 或 1）

# 主检测框（取置信度最高的一个）
float32 center_u       # 框中心像素 u
float32 center_v       # 框中心像素 v
float32 confidence     # 置信度
float32 bbox_x1        # 左上角 x（像素）
float32 bbox_y1        # 左上角 y（像素）
float32 bbox_x2        # 右下角 x（像素）
float32 bbox_y2        # 右下角 y（像素）
