@echo off
title Unified Engineering Workbench - PASS SCIENCE WORKS
echo =========================================================
echo  UNIFIED ENGINEERING WORKBENCH - 1-CLICK LAUNCHER
echo  PASS SCIENCE WORKS ^| R^&D Internship 2026-2027
echo =========================================================
echo.

cd /d "%~dp0"

echo Starting native C++17 REST backend server on port 8080...
start "" "%~dp0workbench_backend.exe" 8080

echo Waiting for backend server initialization...
timeout /t 2 /nobreak >nul

echo Launching engineering dashboard in browser...
start "" http://localhost:8080

echo.
echo Application is live at http://localhost:8080
echo Keep this window open or press any key to close launcher.
pause >nul

