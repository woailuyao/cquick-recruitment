#!/usr/bin/env bash
# 安装 udev 固定设备名（需要 sudo，只做一次）
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULE="99-cquick-servo.rules"

if [[ ! -f "$HERE/$RULE" ]]; then
  echo "找不到 $HERE/$RULE" >&2
  exit 1
fi

sudo cp "$HERE/$RULE" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "已安装 $RULE"
echo "拔插一次 USB-TTL，然后确认："
echo "  ls -l /dev/cquick_servo"
echo "确认后直接 ros2 launch cquick_servo servo.launch.py 即可（port 默认值已是 /dev/cquick_servo）"
