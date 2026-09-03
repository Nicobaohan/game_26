@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0orin_tool.ps1" -Action Deploy
set "TOOL_EXIT=%ERRORLEVEL%"
echo.
if not "%TOOL_EXIT%"=="0" echo Deploy failed. See the error above.
if "%TOOL_EXIT%"=="0" echo Deploy and build completed successfully.
pause
exit /b %TOOL_EXIT%
