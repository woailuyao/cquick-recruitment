#!/usr/bin/env bash
# 给每条命令套一层"姓名 + 实时时间戳"的水印，防止截图被认定无自证。
#
# 用法（在项目根目录）：
#   bash tools/with_stamp.sh python3 tools/run_batch.py
#   bash tools/with_stamp.sh ros2 run cquick_vision target_publisher
#
# 截图时把上面打印的这几行和命令输出一起截进去就行。

set -u
NAME="${CQUICK_NAME:-liuzikang}"

echo "==================== 截图水印 ===================="
echo "姓名 : ${NAME}"
echo "时间 : $(date '+%Y-%m-%d %H:%M:%S %Z')"
echo "主机 : $(hostname)"
echo "目录 : $(pwd)"
echo "命令 : $*"
echo "=================================================="

exec "$@"
