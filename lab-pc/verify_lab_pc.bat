@echo off
rem ==========================================================================
rem  verify_lab_pc.bat: check a lab PC the way a STUDENT sees it.
rem
rem  deploy_lab_pc.bat copies this file to C:\envs. After installing, log out,
rem  log in with a student (standard) account and double-click
rem      C:\envs\verify_lab_pc.bat
rem  A robot window opens for 3 seconds, then a plot window: close the plot.
rem  The last line says whether the PC passed.
rem ==========================================================================
setlocal
title Verify Robotics Lab PC
set "ENV=C:\envs\zuj-robotics"
set FAILED=0

echo.
echo Verifying this PC for Intelligent Mobile Robots (0135343)...
echo.

if not exist "%ENV%\python.exe" (
    echo   [FAIL] The environment is not installed. Run deploy_lab_pc.bat as administrator.
    goto :summary_fail
)

if exist "C:\Users\Public\Desktop\Robotics Lab.bat" (
    echo   [OK]   Robotics Lab shortcut is on the desktop
) else (
    echo   [FAIL] Robotics Lab shortcut is missing from C:\Users\Public\Desktop
    set FAILED=1
)

net session >nul 2>&1
if errorlevel 1 (
    echo   [OK]   This account is a standard account
) else (
    echo   [FAIL] This account has administrator rights: log in as a STUDENT to verify,
    echo          and give students standard accounts so they cannot change the environment
    set FAILED=1
)

rem Students must not be able to write into the environment
(echo test> "%ENV%\write_test.txt") >nul 2>&1
if exist "%ENV%\write_test.txt" (
    del "%ENV%\write_test.txt" >nul 2>&1
    echo   [FAIL] Students can change the environment: C:\envs is not read-only for them
    set FAILED=1
) else (
    echo   [OK]   The environment is read-only for this account
)

rem The same set-up as the Robotics Lab shortcut
set PYTHONNOUSERSITE=1
set PYTHONUTF8=1
set NoDefaultCurrentDirectoryInExePath=
call "%ENV%\Scripts\activate.bat"

echo.
echo Checking the environment (a robot window opens for 3 seconds)...
python C:\envs\check_env.py --gui
if errorlevel 1 set FAILED=1

echo Opening a plot window: CLOSE IT to continue...
python -c "import matplotlib.pyplot as plt; plt.plot([0, 1, 2], [0, 1, 0]); plt.title('Plot window works: close it to continue'); plt.show()"
if errorlevel 1 (
    echo   [FAIL] The plot window could not open
    set FAILED=1
) else (
    echo   [OK]   Plot window
)

if "%FAILED%"=="0" goto :summary_ok

:summary_fail
echo.
echo THIS PC FAILED - see the [FAIL] lines above, and the Troubleshooting
echo section of the setup guide.
echo.
pause
exit /b 1

:summary_ok
echo.
echo This PC passed. Students can use it.
echo.
pause
exit /b 0
