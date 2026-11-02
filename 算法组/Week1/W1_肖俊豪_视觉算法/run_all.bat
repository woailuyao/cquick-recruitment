@echo off
rem ============================================================
rem  Vision Week 1 - one click runner (Windows, CRLF required)
rem
rem  Everything is also written to "run_log.txt" in this folder,
rem  so if the window still closes you can open that file and
rem  see exactly where it stopped.
rem ============================================================
chcp 65001 >nul 2>nul
setlocal
cd /d "%~dp0"
set "PYTHONIOENCODING=utf-8"
set "LOG=run_log.txt"

echo ============================================================
echo  Vision Week 1 - one click runner
echo  Folder: %CD%
echo ============================================================
echo.

> "%LOG%" echo [START] %DATE% %TIME%
>> "%LOG%" echo [FOLDER] %CD%

set "PYEXE="
where python >nul 2>nul
if not errorlevel 1 set "PYEXE=python"
if not defined PYEXE where py >nul 2>nul
if not defined PYEXE if not errorlevel 1 set "PYEXE=py -3"

if defined PYEXE goto havepython

>> "%LOG%" echo [ERROR] Python not found in PATH.
type "%LOG%"
echo.
echo [ERROR] Python was not found.
echo   Install Python 3.10 or newer from https://www.python.org/downloads/
echo   During setup tick "Add python.exe to PATH", then reopen this file.
echo.
pause
exit /b 1

:havepython
echo Using interpreter: %PYEXE%
>> "%LOG%" echo [PYTHON] %PYEXE%
call %PYEXE% -V
call %PYEXE% -V >> "%LOG%" 2>&1
echo.

echo ---- environment check ----
>> "%LOG%" echo.
>> "%LOG%" echo ==== check_env ====
call %PYEXE% "src\check_env.py" >> "%LOG%" 2>&1
if errorlevel 1 goto failed
type "%LOG%" | findstr /B /C:"[Python" /C:"[numpy" /C:"[opencv" /C:"[图片" /C:"[输出" >nul
echo OK

echo ---- task 1: image basics ----
>> "%LOG%" echo.
>> "%LOG%" echo ==== task1_basics ====
call %PYEXE% "src\task1_basics.py" >> "%LOG%" 2>&1

echo ---- task 2: filtering / threshold / edges ----
>> "%LOG%" echo.
>> "%LOG%" echo ==== task2_processing ====
call %PYEXE% "src\task2_processing.py" >> "%LOG%" 2>&1

echo ---- task 3: color detection ----
>> "%LOG%" echo.
>> "%LOG%" echo ==== task3_color_detect ====
call %PYEXE% "src\task3_color_detect.py" --sweep >> "%LOG%" 2>&1

>> "%LOG%" echo.
>> "%LOG%" echo ==== DONE %TIME% ====
goto show

:failed
>> "%LOG%" echo.
>> "%LOG%" echo ==== STOPPED: environment check failed ====

:show
echo.
echo ================= output of this run =================
type "%LOG%"
echo ======================================================
echo.
echo Log saved to: %CD%\%LOG%
echo Results in:   outputs\task1  outputs\task2  outputs\task3
echo.
pause
