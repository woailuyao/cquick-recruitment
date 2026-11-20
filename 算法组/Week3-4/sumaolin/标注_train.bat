@echo off
cd /d "%~dp0"
"tools\venv\Scripts\python.exe" "tools\annotate_markers.py" --split train
pause
