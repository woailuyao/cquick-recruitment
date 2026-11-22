@echo off
rem One-click launcher for labelImg (ASCII only)
rem Opens dataset\pre with classes.txt preloaded.
setlocal
set "EXE=E:\minconda-robot-compute\envs\w34\Scripts\labelImg.exe"
cd /d "%~dp0"
if not exist "%EXE%" (
    echo labelImg.exe not found:
    echo   %EXE%
    echo Install it first:
    echo   E:\minconda-robot-compute\envs\w34\python.exe -m pip install labelImg
    pause
    exit /b 1
)
if not exist "dataset\pre\classes.txt" (
    echo dataset\pre\classes.txt missing. Run tools\prelabel.py first.
    pause
    exit /b 1
)
start "" "%EXE%" "dataset\pre" "dataset\pre\classes.txt"
