@echo off
rem ==========================================================================
rem  deploy_lab_pc.bat: install the shared zuj-robotics environment on ONE lab PC.
rem
rem  Put this folder (with zuj-robotics.tar.gz and check_env.py next to this
rem  script) on a USB stick or network share, then on each lab PC:
rem      right-click deploy_lab_pc.bat -> Run as administrator
rem
rem  Takes about 2 minutes, needs no internet and no Anaconda on the PC.
rem  Running it again replaces the old version (use this to update or repair).
rem  See lab-pc\README.md for how to build zuj-robotics.tar.gz.
rem ==========================================================================
setlocal
set "SRC=%~dp0"
set "ENV=C:\envs\zuj-robotics"

net session >nul 2>&1
if errorlevel 1 (
    echo Please right-click this file and choose "Run as administrator".
    pause & exit /b 1
)
if not exist "%SRC%zuj-robotics.tar.gz" (
    echo zuj-robotics.tar.gz was not found next to this script.
    pause & exit /b 1
)

echo [1/5] Removing any old version and locking C:\envs...
if exist "%ENV%" rmdir /s /q "%ENV%"
if not exist "C:\envs" mkdir "C:\envs"
rem Everything created under C:\envs inherits this: admins can write,
rem student accounts (Users) can only read and run.
icacls "C:\envs" /inheritance:r /grant:r "Administrators:(OI)(CI)F" "SYSTEM:(OI)(CI)F" "Users:(OI)(CI)RX" /Q >nul
if errorlevel 1 ( echo Could not set permissions on C:\envs. & pause & exit /b 1 )
mkdir "%ENV%"

echo [2/5] Extracting the environment (this is the slow step)...
tar -xzf "%SRC%zuj-robotics.tar.gz" -C "%ENV%"
if errorlevel 1 ( echo Extraction failed. & pause & exit /b 1 )

echo [3/5] Fixing paths for this PC...
"%ENV%\Scripts\conda-unpack.exe"
if errorlevel 1 ( echo conda-unpack failed. & pause & exit /b 1 )

echo [4/5] Installing the checker and the desktop shortcut...
copy /y "%SRC%check_env.py" "C:\envs\check_env.py" >nul
copy /y "%SRC%Robotics-Lab.bat" "C:\Users\Public\Desktop\Robotics Lab.bat" >nul

echo [5/5] Testing the installation:
rem Some locked-down Windows images set this, which breaks the activate scripts.
set NoDefaultCurrentDirectoryInExePath=
call "%ENV%\Scripts\activate.bat"
set PYTHONNOUSERSITE=1
set PYTHONUTF8=1
python "C:\envs\check_env.py"
if errorlevel 1 (
    echo DEPLOYMENT FINISHED WITH ERRORS - see the [FAIL] lines above.
) else (
    echo This PC is ready.
)
pause
