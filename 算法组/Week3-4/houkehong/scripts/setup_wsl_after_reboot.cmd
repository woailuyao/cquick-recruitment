@echo off
setlocal
chcp 936 >nul
title WSL2 + Ubuntu 22.04 + ROS2 Humble Setup

echo ============================================================
echo   Run this AFTER the reboot, as Administrator.
echo   Installs / verifies WSL2 + Ubuntu 22.04 + ROS2 Humble.
echo   Log: D:\vision_ws\_setup\wsl_setup.log
echo ============================================================
echo.

net session >nul 2>&1
if errorlevel 1 (
    echo Requesting administrator privileges...
    powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

if not exist "D:\vision_ws\k1_soccer_vision\scripts\setup_wsl.ps1" (
    echo [ERROR] setup_wsl.ps1 not found.
    pause
    exit /b 1
)

powershell -NoProfile -ExecutionPolicy Bypass -File "D:\vision_ws\k1_soccer_vision\scripts\setup_wsl.ps1"
echo.
echo Exit code: %ERRORLEVEL%
echo ============================================================
pause