#!/bin/bash
# 一键编译
#
# 为什么必须显式指定 Python：
#   本机 PATH 里有个 Python 3.13 排在 /usr/bin 前面，CMake 会挑它。
#   但 ROS Humble 的 em / numpy 都装在系统 Python 3.10 下，于是报
#       No module named 'em'
#       execute_process(/path/to/python3.13 -c 'import numpy;...') returned error code 1
#
#   注意要同时指定两个变量（名字不一样，别只写一个）：
#     Python3_EXECUTABLE —— rosidl_adapter / 消息生成用
#     PYTHON_EXECUTABLE  —— rosidl_generator_py 用（不带 3 的那个旧变量）
set -e

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash

PY=/usr/bin/python3

colcon build \
  --packages-select cquick_ball \
  --cmake-args -DPython3_EXECUTABLE=$PY -DPYTHON_EXECUTABLE=$PY

echo "--- 编译完成。新终端请执行:"
echo "    source ~/ros2_ws/install/setup.bash"
