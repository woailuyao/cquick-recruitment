# 足球检测结果（任务 1）
# 发布话题：/soccer/ball
# 坐标约定：图像左上角为原点，u 向右为正、v 向下为正（像素）

std_msgs/Header header
bool detected        # 是否检测到足球
float32 u            # 检测框中心 u（像素）
float32 v            # 检测框中心 v（像素）
float32 confidence   # 置信度 0~1
float32 bbox_l       # 检测框左边界（像素）
float32 bbox_t       # 检测框上边界（像素）
float32 bbox_r       # 检测框右边界（像素）
float32 bbox_b       # 检测框下边界（像素）
uint32 frame_seq     # 处理帧序号（自增）
float32 fps          # 当前处理帧率
