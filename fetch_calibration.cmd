@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0orin_tool.ps1" -Action FetchCalibration
set "TOOL_EXIT=%ERRORLEVEL%"
echo.
if not "%TOOL_EXIT%"=="0" echo Fetch failed. See the error above.
pause
exit /b %TOOL_EXIT%
