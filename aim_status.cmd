@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0orin_tool.ps1" -Action Status
set "TOOL_EXIT=%ERRORLEVEL%"
echo.
pause
exit /b %TOOL_EXIT%
