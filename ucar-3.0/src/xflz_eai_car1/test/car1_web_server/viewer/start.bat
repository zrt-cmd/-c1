@echo off
chcp 65001 >nul
echo ========================================
echo   car1 URDF Viewer - HTTP Server (CORS)
echo ========================================
echo.
echo Starting CORS-enabled HTTP server on port 8080...
echo Open browser: http://localhost:8080/viewer/
echo.
echo Press Ctrl+C to stop the server.
echo ========================================
echo.
cd /d "%~dp0.."
python server_cors.py 8080
