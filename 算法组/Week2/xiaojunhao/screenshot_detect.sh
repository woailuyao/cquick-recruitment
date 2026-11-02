#!/usr/bin/env bash
# 任务 1 的截图用：先在终端打姓名和时间戳，再跑一次检测。
# 截图的时候把这两行一起截进去。
cd "$(dirname "$0")" || exit 1

echo "姓名: xiaojunhao"
date "+%Y-%m-%d %H:%M:%S"
echo

cd src/color_detector || exit 1
python3 -m color_detector.detect_image
