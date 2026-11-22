@echo off
rem CQuick week3-4 one-click runner (ASCII only: cmd.exe reads .bat as OEM codepage)
rem Usage:
rem   run.bat tools\camera_check.py --index 1
rem   run.bat tools\calibrate_intrinsics.py --selftest
rem   run.bat src\sim\test_pipeline.py --trials 120
setlocal
set "PY=E:\minconda-robot-compute\envs\w34\python.exe"
if not exist "%PY%" set "PY=E:\minconda-robot-compute\envs\cv_practice\python.exe"
if not exist "%PY%" set "PY=python"
cd /d "%~dp0"
if "%~1"=="" (
    echo Usage: run.bat ^<script^> [args...]
    echo Example: run.bat tools\camera_check.py --index 1
    exit /b 1
)
"%PY%" %*
set "CODE=%ERRORLEVEL%"
endlocal & exit /b %CODE%
