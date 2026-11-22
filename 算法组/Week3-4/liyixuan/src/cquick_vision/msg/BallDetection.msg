# cquick_interfaces/msg/BallDetection.msg
std_msgs/Header header

string class_name           # 固定为 "ball"
float32 confidence
float32 x1
float32 y1
float32 x2
float32 y2
float32 center_u
float32 center_v
