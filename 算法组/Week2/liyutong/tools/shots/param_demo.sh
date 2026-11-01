#!/usr/bin/env bash
# Shows the real subscriber log before and after a runtime parameter change.
set -u
cd "$(dirname "$0")/../.." || exit 1
rm -f /tmp/param_sub.log

ros2 run color_target_detector target_subscriber > /tmp/param_sub.log 2>&1 &
SUB=$!
ros2 run color_target_detector target_publisher --ros-args \
  -p "images:=[$PWD/images/small.jpg]" \
  -p "hsv_lower:=[5,200,80]" \
  -p "min_area:=100.0" \
  -p "frequency:=2.0" > /tmp/param_pub.log 2>&1 &
PUB=$!

sleep 5
echo "image: images/small.jpg   current min_area:"
ros2 param get /target_publisher min_area
echo
echo "---- before: last 2 received messages ----"
tail -n 2 /tmp/param_sub.log
echo
echo "---- runtime change ----"
ros2 param set /target_publisher min_area 200000.0
sleep 4
echo
echo "---- after: last 3 received messages ----"
tail -n 3 /tmp/param_sub.log

kill "$PUB" "$SUB" 2>/dev/null || true
wait 2>/dev/null || true
echo "nodes stopped"
printf '\xe5\xa7\x93\xe5\x90\x8d: liyutong\n'
date '+%Y-%m-%d %H:%M:%S'
