@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0orin_tool.ps1" -Action StartFire
set "TOOL_EXIT=%ERRORLEVEL%"
echo.
if not "%TOOL_EXIT%"=="0" echo Auto-aim fire mode stopped or failed. See the output above.
pause
exit /b %TOOL_EXIT%
