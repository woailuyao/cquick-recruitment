@echo off
setlocal
chcp 65001 >nul
set "PYTHONUTF8=1"
cd /d "%~dp0.."
if exist ".venv\Scripts\python.exe" set "PATH=%CD%\.venv\Scripts;%PATH%"
python --version
if errorlevel 1 goto missing
set "PROMPT=W1 $G "
echo Run python src/record_step.py 1 to start the screenshot steps.
cmd /k
exit /b
:missing
echo Please activate a Python 3.10 environment and install src/requirements.txt.
pause