#!/usr/bin/env bash
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROS_WS:-$HOME/ros2_ws}"
PORT="${PORT:-9000}"
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "找不到 $WS/install/setup.bash" >&2
  echo "先编译: cd \"$WS\" && colcon build --packages-select field_pose_msgs && source install/setup.bash" >&2
  echo "或设 ROS_WS 指向你的工作区" >&2
  exit 1
fi
source "$WS/install/setup.bash"
echo "桥接启动: $HERE/field_pose_node.py port=$PORT"
echo "Windows 端: python src/p3p.py 1 --send <本VM的IP>:$PORT"
exec python3 "$HERE/field_pose_node.py" --port "$PORT"