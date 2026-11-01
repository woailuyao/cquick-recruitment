$ErrorActionPreference = 'Continue'
$Host.UI.RawUI.WindowTitle = 'liyutong-shot-01'
Set-Location 'C:\Users\liyut\Documents\deepseek-harness\default-workspace\week2_color_target'
chcp 65001 | Out-Null

$cmd = "printf '\xe5\xa7\x93\xe5\x90\x8d: liyutong\n'; date '+%Y-%m-%d %H:%M:%S'; ros2 run color_target_detector detect_images images/clear.jpg images/small.jpg images/background.jpg images/lighting.jpg images/absent.jpg --output outputs/screenshot_detection --hsv-lower 5 200 80 --min-area 100"

wsl -d Ubuntu-22.04 --cd /mnt/c/Users/liyut/Documents/deepseek-harness/default-workspace/week2_color_target -- bash -lc "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; $cmd"

Write-Host ''
Write-Host '==== detection finished ===='
Start-Sleep -Seconds 900
