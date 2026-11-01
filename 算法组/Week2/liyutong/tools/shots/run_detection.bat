@echo off
chcp 65001 >nul
title liyutong week2 detection
wsl -d Ubuntu-22.04 --cd /mnt/c/Users/liyut/Documents/deepseek-harness/default-workspace/week2_color_target -- bash -lc "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; echo '姓名: liyutong'; date '+%%Y-%%m-%%d %%H:%%M:%%S'; ros2 run color_target_detector detect_images images/clear.jpg images/small.jpg images/background.jpg images/lighting.jpg images/absent.jpg --output outputs/screenshot_detection --hsv-lower 5 200 80 --min-area 100"
echo.
echo ==== command finished ====
pause >nul
