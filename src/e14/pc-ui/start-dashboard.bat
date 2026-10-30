@echo off
setlocal
cd /d "%~dp0"

where node.exe >nul 2>nul
if errorlevel 1 if exist "%ProgramFiles%\nodejs\node.exe" set "PATH=%ProgramFiles%\nodejs;%PATH%"
where node.exe >nul 2>nul
if errorlevel 1 (
    echo Node.js was not found. Install Node.js 18 or newer, then reopen this window.
    pause
    exit /b 1
)

where npm.cmd >nul 2>nul
if errorlevel 1 if exist "%ProgramFiles%\nodejs\npm.cmd" set "PATH=%ProgramFiles%\nodejs;%PATH%"
where npm.cmd >nul 2>nul
if errorlevel 1 (
    echo npm was not found. Repair the Node.js installation and try again.
    pause
    exit /b 1
)

if not exist "node_modules\serialport\package.json" (
    echo Dashboard dependencies are missing.
    echo Open a terminal in this folder and run: npm install
    pause
    exit /b 1
)

set "MODEM_PORT=%~1"
if not defined MODEM_PORT (
    echo Available serial ports:
    node.exe list-serial-ports.mjs
    echo.
    set /p "MODEM_PORT=Modem COM port [COM15]: "
)
if not defined MODEM_PORT set "MODEM_PORT=COM15"

echo Starting E14 dashboard at http://127.0.0.1:8080 using modem %MODEM_PORT%
echo Keep this window open while using the dashboard. Press Ctrl+C to stop it.
call npm.cmd start
set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" pause
exit /b %EXIT_CODE%
