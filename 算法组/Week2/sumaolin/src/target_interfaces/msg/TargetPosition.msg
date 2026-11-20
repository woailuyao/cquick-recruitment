# 目标位置消息
#
# 约定：detected=False 时 u/v/area 一律填 0，但接收端判断有效性只看 detected，
# 不能用 u==0 去推断"没检测到"——因为 (0,0) 本身是合法像素坐标（图像左上角）。

bool detected

# 目标中心在图像中的像素坐标。原点在左上角，u 向右增大，v 向下增大。
float32 u
float32 v

# 目标轮廓面积（像素平方），可粗略反映目标远近和这一帧的可信度。
float32 area
