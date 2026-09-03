@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0orin_tool.ps1" -Action Sync -OpenCode
set "SYNC_EXIT=%ERRORLEVEL%"
if not "%SYNC_EXIT%"=="0" (
    echo.
    echo Sync failed. See the error above.
)
echo.
pause
exit /b %SYNC_EXIT%
