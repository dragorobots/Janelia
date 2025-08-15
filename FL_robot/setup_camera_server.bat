@echo off
REM Setup Camera Server on Robot (Windows)
REM Run this script on the robot to start the camera server

echo 🎥 Setting up Robot Camera Server
echo ==================================

REM Check if we're on the robot (you can modify this check)
if not exist "C:\opt\ros\humble\setup.bat" (
    echo ⚠️  Warning: This doesn't appear to be a ROS2 robot environment
    echo    Make sure you're running this on the robot!
)

REM Install required packages if not already installed
echo 📦 Installing required packages...
pip install flask opencv-python numpy

REM Check if camera is available
echo 📷 Checking camera availability...
python -c "import cv2; cap = cv2.VideoCapture(0); print('✅ Camera 0 is available' if cap.isOpened() else '❌ Camera 0 is not available'); cap.release()"

echo.
echo 🚀 Starting camera server...
echo    The server will be available at:
echo    - http://localhost:8080/
echo    - http://YOUR_ROBOT_IP:8080/stream.mjpg
echo.
echo    Press Ctrl+C to stop the server
echo.

REM Start the camera server
python camera_server.py --port 8080 --quality 80

pause
