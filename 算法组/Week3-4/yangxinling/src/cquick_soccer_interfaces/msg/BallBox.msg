# =============================================================================
# BallBox.msg —— 单个检测框（多球/调试时用）
# 主话题 /soccer/ball 用 TargetPosition（单目标、模拟比赛里只有一颗球）；
# 需要一次看到所有候选时用 /soccer/ball_boxes（BallBox[]）。
# =============================================================================

float32 confidence
float32[4] bbox          # [x1, y1, x2, y2]
uint32 class_id
string class_name
