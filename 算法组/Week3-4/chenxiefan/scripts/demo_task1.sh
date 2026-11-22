#!/bin/bash
cd ~/ros2_ws/week3_4 || exit 1
export QT_QPA_FONTDIR=/usr/share/fonts/truetype/dejavu
SRC="${1:-http://192.168.56.1:8080/video}"
exec python3 scripts/detect_ball.py --model models/ball_best.pt --source "$SRC" --conf 0.4
