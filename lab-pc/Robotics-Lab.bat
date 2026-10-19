@echo off
rem ==========================================================================
rem  Robotics Lab: opens a terminal with the shared zuj-robotics environment
rem  already active, then checks the environment.
rem
rem  deploy_lab_pc.bat copies this file to the Public Desktop of each lab PC,
rem  so every student account sees it. Students never type an environment name.
rem ==========================================================================
title Robotics Lab (zuj-robotics)

set "ENV=C:\envs\zuj-robotics"
if not exist "%ENV%\python.exe" (
    echo The robotics environment is not installed on this PC.
    echo Please tell the instructor, and use the Colab version of the lab for now.
    pause
    exit /b 1
)

rem Ignore "pip install --user" leftovers that could replace the course packages.
set PYTHONNOUSERSITE=1
rem UTF-8 mode: saving lab output to a file otherwise fails on Windows.
set PYTHONUTF8=1

rem Some locked-down Windows images set this, which breaks the activate scripts.
set NoDefaultCurrentDirectoryInExePath=
call "%ENV%\Scripts\activate.bat"
cd /d "%USERPROFILE%\Documents"

cmd /k "python C:\envs\check_env.py & echo Your work is NOT kept on this PC: push it to GitHub or upload it before you leave. & echo."
