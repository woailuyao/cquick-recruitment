param(
    [Parameter(Mandatory=$true)][string]$Title,
    [Parameter(Mandatory=$true)][string]$Bash,
    [int]$KeepSeconds = 900
)
$ErrorActionPreference = 'Continue'
$Host.UI.RawUI.WindowTitle = $Title
$size = $Host.UI.RawUI.WindowSize
$size.Width = 120
$size.Height = 34
$Host.UI.RawUI.WindowSize = $size
Set-Location 'C:\Users\liyut\Documents\deepseek-harness\default-workspace\week2_color_target'
chcp 65001 | Out-Null

$full = "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; printf '\xe5\xa7\x93\xe5\x90\x8d: liyutong\n'; date '+%Y-%m-%d %H:%M:%S'; $Bash"
wsl -d Ubuntu-22.04 --cd /mnt/c/Users/liyut/Documents/deepseek-harness/default-workspace/week2_color_target -- bash -lc "$full"

Write-Host ''
Write-Host '==== finished ===='
Start-Sleep -Seconds $KeepSeconds
