# 足球检测结果（对应细则 2.11 的 /soccer/ball）
# 单位：像素（bbox / center），置信度 0~1；detected=false 时其余字段无意义
std_msgs/Header header
bool detected
float32 conf
int32 x1
int32 y1
int32 x2
int32 y2
float32 center_u
float32 center_v
