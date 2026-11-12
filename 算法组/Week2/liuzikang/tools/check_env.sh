#!/usr/bin/env bash
# 一键打印所有版本信息，用来填 README 的"环境版本"表格，也可以直接截图当证据。
#
# 用法：
#   bash tools/check_env.sh

echo "==================== 环境自检 ===================="
echo -n "Ubuntu   : "; (lsb_release -ds 2>/dev/null || grep PRETTY_NAME /etc/os-release | cut -d= -f2 | tr -d '"')
echo -n "内核     : "; uname -r
echo -n "ROS_DISTRO : "; echo "${ROS_DISTRO:-<空！说明没有 source ROS2>}"
echo -n "Python   : "; python3 --version
echo -n "OpenCV   : "; python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null || echo "<没装，sudo apt install python3-opencv>"
echo -n "NumPy    : "; python3 -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "<没装>"
echo -n "cv2 路径 : "; python3 -c "import cv2; print(cv2.__file__)" 2>/dev/null || echo "-"
echo -n "colcon   : "; (colcon version-check >/dev/null 2>&1 && echo "已安装") || echo "<没装，sudo apt install python3-colcon-common-extensions>"
echo -n "git      : "; git --version 2>/dev/null || echo "<没装，sudo apt install git>"
echo "=================================================="
if [ "${ROS_DISTRO:-}" != "humble" ]; then
  echo "!! 注意：ROS_DISTRO 不是 humble。每个新终端都要执行："
  echo "   source /opt/ros/humble/setup.bash"
fi
