#!/usr/bin/env bash
# Message definition, live topic type, and one real message, all while the publisher runs.
set -u
cd "$(dirname "$0")/../.." || exit 1

ros2 run color_target_detector target_publisher --ros-args \
  -p "images:=[$PWD/images/clear.jpg]" \
  -p "hsv_lower:=[5,200,80]" \
  -p "min_area:=100.0" \
  -p "frequency:=1.0" \
  -p "max_messages:=30" > /tmp/topic_pub.log 2>&1 &
PUB=$!
sleep 3

echo "---- ros2 interface show ----"
ros2 interface show target_interfaces/msg/TargetPosition
echo
echo "---- ros2 topic info ----"
ros2 topic info /target_position
echo
echo "---- one live message ----"
ros2 topic echo /target_position target_interfaces/msg/TargetPosition --once

kill "$PUB" 2>/dev/null || true
wait "$PUB" 2>/dev/null || true
echo "publisher stopped"
printf '\xe5\xa7\x93\xe5\x90\x8d: liyutong\n'
date '+%Y-%m-%d %H:%M:%S'
