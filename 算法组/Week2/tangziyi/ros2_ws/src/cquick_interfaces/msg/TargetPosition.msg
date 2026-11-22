# CQuick vision group - Week 2 - Task 3
# 目标位置（图像像素坐标）
#
# 重要：u、v 是图像像素坐标
#   - 原点是图像左上角 (0, 0)
#   - u 向右增大，v 向下增大
#   - 不是机器人坐标，也不是米制距离
#   - 转成场地坐标需要相机标定，本周不做
#
# detected == False 时，u / v / area 约定全为 0。
# 使用前必须先判断 detected，不能把 (0,0) 当成"目标在左上角"。

bool detected        # 本帧是否检测到目标
float32 u            # 目标中心横坐标（像素）
float32 v            # 目标中心纵坐标（像素）
float32 area         # 目标轮廓面积（像素平方，近似值）
