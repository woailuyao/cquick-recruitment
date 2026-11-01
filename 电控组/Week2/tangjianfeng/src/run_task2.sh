#!/bin/bash
# 任务2 · 一键复现脚本
#   1) 启动 turtlesim（离屏渲染，不需要显示器）
#   2) 清屏（服务调用，只调一次有回应）
#   3) 启动任务1 的发布者（2 Hz 发 3 个球）
#   4) 启动导航节点，走完 1->2->3
#   5) 导出画布 SVG，并打印完整日志
set +u

OUT_DIR="${1:-/home/miku/shots}"
mkdir -p "$OUT_DIR"
source /opt/ros/humble/setup.bash
source /home/miku/ros2_ws/install/setup.bash

# 清理可能存在的旧实例
pkill -f turtlesim_node 2>/dev/null
pkill -f ball_publisher 2>/dev/null
pkill -f ball_navigator 2>/dev/null
sleep 1

QT_QPA_PLATFORM=offscreen ros2 run turtlesim turtlesim_node \
  --ros-args -p background_r:=255 -p background_g:=255 -p background_b:=255 \
  > "$OUT_DIR/turtlesim.log" 2>&1 &
TURTLE_PID=$!
sleep 4

if ! kill -0 $TURTLE_PID 2>/dev/null; then
  echo "[FAIL] turtlesim 没起来"; tail -20 "$OUT_DIR/turtlesim.log"; exit 1
fi
echo "[OK] turtlesim 已启动 (pid=$TURTLE_PID)"

# --- 清屏：一次服务调用 ---
echo "--- 清屏（service call）"
ros2 service call /clear std_srvs/srv/Empty >/dev/null 2>&1 && echo "[OK] /clear 已调用"

# 设置笔：红色，宽 3
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen \
  "{r: 220, g: 30, b: 30, width: 3, off: 0}" >/dev/null 2>&1

# --- 发布者 ---
ros2 run cquick_ball ball_publisher > "$OUT_DIR/publisher.log" 2>&1 &
PUB_PID=$!

# 轨迹记录器：把乌龟真实位姿写成 CSV
/usr/bin/python3 /home/miku/ros2_ws/tools/record_pose.py "$OUT_DIR/pose.csv" \
  > "$OUT_DIR/recorder.log" 2>&1 &
REC_PID=$!
sleep 1

echo "--- topic info"
ros2 topic info /ball_pos

echo "--- node list (导航前)"
ros2 node list

# --- 导航节点 ---
ros2 run cquick_ball ball_navigator > "$OUT_DIR/navigator.log" 2>&1 &
NAV_PID=$!

# 等导航结束（最多 120 秒）
for i in $(seq 1 240); do
  if grep -q "全部 3 个点已到达" "$OUT_DIR/navigator.log" 2>/dev/null; then
    echo "[OK] 导航完成，用时约 $((i/2)) 秒"
    break
  fi
  sleep 0.5
done

sleep 1
echo "--- node list (导航后)"
ros2 node list

echo "--- 最终位姿"
ros2 topic echo /turtle1/pose --once 2>/dev/null | head -4

# 收尾
kill $PUB_PID $NAV_PID $REC_PID 2>/dev/null
sleep 1
kill $TURTLE_PID 2>/dev/null
wait 2>/dev/null
echo "[DONE]"
