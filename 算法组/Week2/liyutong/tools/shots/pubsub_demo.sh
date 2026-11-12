#!/usr/bin/env bash
# Both real nodes run simultaneously; output interleaves in one console for the split-view shot.
set -u
cd "$(dirname "$0")/../.." || exit 1
ros2 run color_target_detector target_subscriber &
SUB=$!
sleep 3
ros2 run color_target_detector target_publisher --ros-args \
  -p "images:=[$PWD/images/clear.jpg,$PWD/images/absent.jpg]" \
  -p "hsv_lower:=[5,200,80]" \
  -p "min_area:=100.0" \
  -p "max_messages:=14"
sleep 2
kill "$SUB" 2>/dev/null || true
wait "$SUB" 2>/dev/null || true
echo "both nodes stopped"
printf '\xe5\xa7\x93\xe5\x90\x8d: liyutong\n'
date '+%Y-%m-%d %H:%M:%S'
