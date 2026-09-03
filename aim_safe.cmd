@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0orin_tool.ps1" -Action StartSafe
set "TOOL_EXIT=%ERRORLEVEL%"
echo.
if not "%TOOL_EXIT%"=="0" echo Safe auto-aim stopped or failed. See the output above.
pause
exit /b %TOOL_EXIT%
